#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import glob
import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import torch

# 너가 사용하던 LocalMap 로직 그대로 재사용
from local_occ_grid_map import LocalMap

# --------- Map params (메인 노드와 동일하게 맞춰야 함) ----------
IMG_SIZE = 64
SEQ_LEN = 10
P_prior = 0.5
MAP_X_LIMIT = [0, 6.4]
MAP_Y_LIMIT = [-3.2, 3.2]
RESOLUTION = 0.1
# ---------------------------------------------------------------

def build_history_binary_maps(scans, positions, velocities, device="cpu"):
    """
    scans:      (T,1080) float32
    positions:  (T,3)    float32
    velocities: (T,2)    float32
    return: history_maps (T, H, W) float32 in {0,1}
    """
    T = scans.shape[0]
    batch_size = 1

    scans_t = torch.from_numpy(scans).float().unsqueeze(0).to(device)         # (1,T,1080)
    pos_t   = torch.from_numpy(positions).float().unsqueeze(0).to(device)     # (1,T,3)
    vel_t   = torch.from_numpy(velocities).float().unsqueeze(0).to(device)    # (1,T,2)

    # LocalMap 생성
    gridMap = LocalMap(
        X_lim=MAP_X_LIMIT,
        Y_lim=MAP_Y_LIMIT,
        resolution=RESOLUTION,
        p=P_prior,
        size=[batch_size, T],
        device=device
    )

    # 메인 노드와 동일하게 "예측 reference frame" 설정
    obs_pos_N = pos_t[:, T-1]    # (1,3)
    vel_N     = vel_t[:, T-1]    # (1,2)

    pred_T = 6
    noise_std = [0,0,0]
    pos_origin = gridMap.origin_pose_prediction(vel_N, obs_pos_N, pred_T, noise_std)

    # past poses -> predicted frame
    pos = pos_t[:, :T]  # (1,T,3)
    x_odom, y_odom, theta_odom = gridMap.robot_coordinate_transform(pos, pos_origin)

    distances = scans_t[:, :T]   # (1,T,1080)

    angles = torch.linspace(-(135*np.pi/180), 135*np.pi/180, distances.shape[-1]).to(device)
    dx, dy = gridMap.lidar_scan_xy(distances, angles, x_odom, y_odom, theta_odom)

    # discretize -> (1,T,H,W)
    binary_maps = gridMap.discretize(dx, dy)   # usually float tensor 0/1

    # return (T,H,W) numpy
    return binary_maps[0].detach().cpu().numpy()


def render_one(npz_path, out_dir, device="cpu", scale=4):
    data = np.load(npz_path, allow_pickle=True)

    scans = data["scans"]    # (T,1080)
    pos   = data["pos"]      # (T,3)
    vel   = data["vel"]      # (T,2)

    # ---------------- local_map load + orientation fix ----------------
    local_map = data["local_map"]  # (H,W) int16 [-1..100]
    lm = local_map.astype(np.float32)
    lm[lm < 0] = 0
    lm = np.clip(lm, 0, 100) / 100.0  # (H,W) in [0,1]

    # 메인 publish에서 transpose를 넣었기 때문에 후처리에서 복원
    lm = lm.T

    # 만약 위아래가 뒤집힌 느낌이면 아래 한 줄도 켜봐
    # lm = np.flipud(lm)
    # -----------------------------------------------------------------

    # build history maps
    hist_maps = build_history_binary_maps(scans, pos, vel, device=device)  # (T,H,W)
    T = hist_maps.shape[0]
    H, W = hist_maps.shape[1], hist_maps.shape[2]

    # ---------------- insert separators between frames ----------------
    gap = 2           # separator thickness in pixels
    gap_val = 0.35    # separator intensity in [0,1] (0=black, 1=white)

    sep = np.ones((H, gap), dtype=np.float32) * gap_val

    frames = []
    for t in range(T):
        frames.append(hist_maps[t].astype(np.float32))
        if t != T - 1:
            frames.append(sep)

    hist_row = np.concatenate(frames, axis=1)  # (H, W*T + gap*(T-1))

    # history와 local_map 사이 separator 추가
    combo = np.concatenate([hist_row, sep, lm.astype(np.float32)], axis=1)
    # -----------------------------------------------------------------

    # upscale (nearest) for readability
    combo_u8 = (combo * 255).astype(np.uint8)
    combo_vis = np.repeat(np.repeat(combo_u8, scale, axis=0), scale, axis=1)

    # plot with matplotlib (just for title text + save)
    fig = plt.figure(figsize=(12, 2))
    plt.imshow(combo_vis, cmap="gray", vmin=0, vmax=255)
    plt.axis("off")

    plt.title(f"[history x{T}] | [local_map] : {os.path.basename(npz_path)}", fontsize=8)

    os.makedirs(out_dir, exist_ok=True)
    base = os.path.splitext(os.path.basename(npz_path))[0]
    out_path = os.path.join(out_dir, f"{base}_render.png")
    plt.savefig(out_path, dpi=200, bbox_inches="tight", pad_inches=0.02)
    plt.close(fig)

    return out_path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--npz", type=str, required=True,
                    help="input npz file OR directory (if dir, render all npz)")
    ap.add_argument("--out", type=str, default="./render_out",
                    help="output directory for png renders")
    ap.add_argument("--device", type=str, default="cpu", choices=["cpu","cuda"],
                    help="device for LocalMap ops")
    ap.add_argument("--scale", type=int, default=4, help="upscale factor")
    args = ap.parse_args()

    paths = []
    if os.path.isdir(args.npz):
        paths = sorted(glob.glob(os.path.join(args.npz, "*.npz")))
    else:
        paths = [args.npz]

    if len(paths) == 0:
        print("No npz found.")
        return

    for p in paths:
        out_path = render_one(p, args.out, device=args.device, scale=args.scale)
        print("saved:", out_path)

if __name__ == "__main__":
    main()
