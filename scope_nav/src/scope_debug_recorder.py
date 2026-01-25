#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import threading
import queue
from collections import deque

import rospy
import numpy as np

from scope_msgs.msg import ScopeInputData
from nav_msgs.msg import OccupancyGrid

import message_filters


class ScopeDebugRecorder:
    def __init__(self):
        rospy.init_node("scope_debug_recorder", anonymous=True)

        # -------------------- params --------------------
        self.seq_len = rospy.get_param("~seq_len", 10)
        self.img_size = rospy.get_param("~img_size", 64)

        self.input_topic = rospy.get_param("~input_topic", "/scope_input_data")
        self.map_topic   = rospy.get_param("~map_topic",   "/scope_nav/local_map")  # or /local_map

        self.out_dir = rospy.get_param("~out_dir", os.path.join(os.path.expanduser("~"), "scope_debug_dump"))
        os.makedirs(self.out_dir, exist_ok=True)

        # approximate sync settings
        self.slop = rospy.get_param("~slop", 0.20)  # seconds allowed between stamps
        self.sync_queue = rospy.get_param("~sync_queue", 50)

        # drop policy
        self.max_pending_jobs = rospy.get_param("~max_pending_jobs", 200)  # if too many save jobs pending, drop
        self.max_hist_buffer  = rospy.get_param("~max_hist_buffer", 300)   # keep limited number of input frames

        # saving frequency (optional throttle)
        self.save_every = rospy.get_param("~save_every", 1)  # 1=save all matched pairs, 5=save every 5th

        # -------------------- buffers --------------------
        # store latest input frames to build history
        self.hist = deque(maxlen=self.seq_len)     # holds tuples (stamp, scan_ranges, curr_pos, curr_vel, curr_odom)

        # store extra frames just in case
        self.input_frame_buffer = deque(maxlen=self.max_hist_buffer)

        # async saving
        self.job_q = queue.Queue()
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker.start()

        self.save_cnt = 0
        self.match_cnt = 0

        # -------------------- subscribers + sync --------------------
        input_sub = message_filters.Subscriber(self.input_topic, ScopeInputData)
        map_sub   = message_filters.Subscriber(self.map_topic, OccupancyGrid)

        # Approximate sync: pairs the closest timestamps within slop
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [input_sub, map_sub],
            queue_size=self.sync_queue,
            slop=self.slop,
            allow_headerless=False
        )
        self.sync.registerCallback(self.synced_cb)

        rospy.loginfo("[scope_debug_recorder] recording started")
        rospy.loginfo(" input_topic=%s", self.input_topic)
        rospy.loginfo(" map_topic  =%s", self.map_topic)
        rospy.loginfo(" out_dir    =%s", self.out_dir)
        rospy.loginfo(" slop=%.3f sec, save_every=%d", self.slop, self.save_every)

    # -------------------------------------------------------
    # When input and local_map are time-matched, this callback triggers.
    # Keep it LIGHT: only package + enqueue job.
    # -------------------------------------------------------
    def synced_cb(self, input_msg, map_msg):
        self.match_cnt += 1

        # Throttle saving if desired
        if (self.match_cnt % self.save_every) != 0:
            return

        # ---------------- Build history from recent input frames ----------------
        # NOTE: This is NOT exactly the same 10-frame set used by your main model unless:
        #  - your main model uses the same stamps and publishes local_map with same stamp,
        #  - or your scope_input_data is aligned to the inference cycle.
        #
        # But it is still VERY useful to diagnose occlusion:
        # you can see whether scan_ranges contain anything during the last 10 frames.

        # Add current input frame to buffer
        stamp = input_msg.header.stamp.to_sec()
        frame = (
            stamp,
            np.array(input_msg.scan_ranges, dtype=np.float32),
            np.array(input_msg.curr_pos, dtype=np.float32),
            np.array(input_msg.curr_vel, dtype=np.float32),
            np.array(input_msg.curr_odom, dtype=np.float32),
        )
        self.input_frame_buffer.append(frame)

        # Grab the latest seq_len frames (best effort)
        if len(self.input_frame_buffer) < self.seq_len:
            # not enough history yet
            return

        hist_frames = list(self.input_frame_buffer)[-self.seq_len:]  # list of tuples

        # ---------------- Extract local_map raw grid ----------------
        H = map_msg.info.height
        W = map_msg.info.width
        grid = np.array(map_msg.data, dtype=np.int16).reshape(H, W)

        # package job
        job = {
            "stamp_input": stamp,
            "stamp_map": map_msg.header.stamp.to_sec(),
            "hist_frames": hist_frames,  # list length seq_len
            "grid": grid,
            "map_info": {
                "resolution": map_msg.info.resolution,
                "origin_x": map_msg.info.origin.position.x,
                "origin_y": map_msg.info.origin.position.y,
                "frame_id": map_msg.header.frame_id,
                "width": W,
                "height": H
            }
        }

        # drop if backlog is too large
        if self.job_q.qsize() > self.max_pending_jobs:
            rospy.logwarn_throttle(2.0, "[scope_debug_recorder] save backlog too big (%d). Dropping.", self.job_q.qsize())
            return

        self.job_q.put(job)

    # -------------------------------------------------------
    # Worker thread: does actual disk I/O (slow part)
    # -------------------------------------------------------
    def _worker_loop(self):
        while not rospy.is_shutdown():
            try:
                job = self.job_q.get(timeout=0.5)
            except queue.Empty:
                continue

            try:
                self._save_job(job)
            except Exception as e:
                rospy.logwarn("[scope_debug_recorder] save error: %s", str(e))

            self.job_q.task_done()

    def _save_job(self, job):
        # file name tied to map stamp (more meaningful for debugging)
        ts = job["stamp_map"]
        fname = f"dbg_{ts:.3f}_{self.save_cnt:06d}.npz"
        path = os.path.join(self.out_dir, fname)

        # unpack history
        # hist_frames: [(stamp, scan_ranges, curr_pos, curr_vel, curr_odom), ...]
        stamps = np.array([f[0] for f in job["hist_frames"]], dtype=np.float64)
        scans  = np.stack([f[1] for f in job["hist_frames"]], axis=0)  # (T,1080)
        pos    = np.stack([f[2] for f in job["hist_frames"]], axis=0)  # (T,3)
        vel    = np.stack([f[3] for f in job["hist_frames"]], axis=0)  # (T,2)
        odom   = np.stack([f[4] for f in job["hist_frames"]], axis=0)  # (T,3)

        # save raw (fast, reliable)
        np.savez_compressed(
            path,
            stamps=stamps,
            scans=scans,
            pos=pos,
            vel=vel,
            odom=odom,
            local_map=job["grid"],
            map_resolution=job["map_info"]["resolution"],
            map_origin_x=job["map_info"]["origin_x"],
            map_origin_y=job["map_info"]["origin_y"],
            map_frame_id=job["map_info"]["frame_id"],
        )

        self.save_cnt += 1
        if (self.save_cnt % 20) == 0:
            rospy.loginfo("[scope_debug_recorder] saved %d files (latest: %s)", self.save_cnt, path)


if __name__ == "__main__":
    ScopeDebugRecorder()
    rospy.spin()
