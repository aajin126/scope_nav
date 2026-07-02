#!/usr/bin/env python3

import argparse
import csv
import json
import math
import os
import signal
import subprocess
import sys
import time

import actionlib
import rospy
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


RESULT_PREFIX = "RESULT_JSON:"


class SameGoalEvaluator:
    def __init__(self, args):
        self.args = args

        self.odom_msg = None
        self.prev_xy = None
        self.path_length = 0.0
        self.trial_active = False
        self.global_plan_times_sec = []
        self.collided = False

        rospy.init_node("same_goal_eval_worker", anonymous=True)
        rospy.set_param("/use_sim_time", args.use_sim_time)


        rospy.Subscriber(args.odom_topic, Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber(args.global_plan_time_topic, Float64, self.global_plan_time_cb, queue_size=50)

        if args.collision_topic:
            self.subscribe_collision(args.collision_topic)

        self.move_base = actionlib.SimpleActionClient(args.move_base_action, MoveBaseAction)

    def subscribe_collision(self, topic):
        try:
            from kobuki_msgs.msg import BumperEvent
            rospy.Subscriber(topic, BumperEvent, self.bumper_collision_cb, queue_size=10)
            rospy.loginfo("collision checking enabled: %s", topic)
        except ImportError:
            rospy.logwarn("kobuki_msgs is not available. Collision checking is disabled.")

    def odom_cb(self, msg):
        self.odom_msg = msg

        if not self.trial_active:
            return

        xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.prev_xy is not None:
            step = distance(self.prev_xy, xy)
            if step < self.args.max_odom_step:
                self.path_length += step
        self.prev_xy = xy

    def global_plan_time_cb(self, msg):
        if self.trial_active:
            self.global_plan_times_sec.append(float(msg.data))

    def bumper_collision_cb(self, msg):
        if getattr(msg, "state", 0) == 1:
            self.collided = True

    def wait_ready(self):
        rospy.loginfo("waiting for odom: %s", self.args.odom_topic)
        rospy.wait_for_message(self.args.odom_topic, Odometry, timeout=5.0)

        rospy.loginfo("waiting for move_base action server: %s", self.args.move_base_action)
        self.move_base.wait_for_server()

    def make_goal(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.args.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.args.goal_x
        goal.target_pose.pose.position.y = self.args.goal_y
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation = yaw_to_quaternion(self.args.goal_yaw)
        return goal

    def distance_to_goal(self, goal):
        if self.odom_msg is None:
            return float("inf")
        p = self.odom_msg.pose.pose.position
        g = goal.target_pose.pose.position
        return math.hypot(p.x - g.x, p.y - g.y)

    def run_trial(self, trial):
        goal = self.make_goal()

        self.move_base.cancel_all_goals()
        rospy.sleep(0.2)

        rospy.loginfo(
            "trial=%d send goal: frame=%s x=%.3f y=%.3f yaw=%.3f",
            trial,
            goal.target_pose.header.frame_id,
            goal.target_pose.pose.position.x,
            goal.target_pose.pose.position.y,
            self.args.goal_yaw,
        )

        self.trial_active = True
        self.prev_xy = None
        self.path_length = 0.0
        self.global_plan_times_sec = []
        self.collided = False

        self.move_base.send_goal(goal)

        start_time = rospy.Time.now().to_sec()
        status = "failed"
        travel_time = -1.0

        rate = rospy.Rate(self.args.check_rate)
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            elapsed = now - start_time

            if self.distance_to_goal(goal) <= self.args.goal_tolerance:
                status = "succeeded"
                travel_time = elapsed
                break

            if self.collided:
                status = "collided"
                travel_time = elapsed
                break

            if elapsed >= self.args.timeout_sec:
                status = "timeout"
                travel_time = elapsed
                break

            rate.sleep()

        self.trial_active = False
        self.move_base.cancel_goal()
        rospy.sleep(0.2)

        plan_count = len(self.global_plan_times_sec)
        mean_plan_time = -1.0
        max_plan_time = -1.0
        if plan_count > 0:
            mean_plan_time = sum(self.global_plan_times_sec) / plan_count
            max_plan_time = max(self.global_plan_times_sec)

        avg_speed = 0.0
        if travel_time > 0.0:
            avg_speed = self.path_length / travel_time

        return {
            "trial": trial,
            "status": status,
            "travel_time_sec": travel_time,
            "path_length": self.path_length,
            "avg_speed": avg_speed,
            "global_plan_time_mean": mean_plan_time,
            "global_plan_time_max": max_plan_time,
            "global_plan_call_count": plan_count,
        }


def distance(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)
    return q


def parse_launch_args(items):
    launch_args = []
    for item in items:
        if item.startswith("--"):
            raise ValueError("launch args should be passed after --launch-arg, e.g. --launch-arg gui:=false")
        launch_args.append(item)
    return launch_args


def has_launch_arg(launch_args, name):
    prefix = name + ":="
    return any(arg.startswith(prefix) for arg in launch_args)


def add_gazebo_recording_args(args, trial_dir):
    launch_args = list(args.launch_args)
    if not args.gazebo_recording:
        return launch_args

    if not has_launch_arg(launch_args, "gazebo_recording"):
        launch_args.append("gazebo_recording:=true")

    if not has_launch_arg(launch_args, "gazebo_record_args"):
        record_dir = os.path.expanduser(args.gazebo_record_dir)
        if not os.path.isabs(record_dir):
            record_dir = os.path.join(trial_dir, record_dir)
        os.makedirs(record_dir, exist_ok=True)
        launch_args.append("gazebo_record_args:=--record_path %s" % record_dir)

    return launch_args

def add_rosbag_args(args, trial_dir, trial):
    launch_args = list(args.launch_args)

    if not has_launch_arg(launch_args, "bag_path"):
        bag_name = args.bag_name_format % trial
        bag_path = os.path.join(trial_dir, bag_name)
        launch_args.append("bag_path:=%s" % bag_path)

    return launch_args

def build_launch_cmd(args, trial_dir, trial):
    if args.launch_file.endswith(".launch") and os.path.isabs(args.launch_file):
        cmd = ["roslaunch", args.launch_file]
    else:
        cmd = ["roslaunch", args.launch_package, args.launch_file]

    cmd += add_gazebo_recording_args(args, trial_dir)
    cmd += add_rosbag_args(args, trial_dir, trial)
    return cmd


def start_launch(args, trial_dir, trial):
    cmd = build_launch_cmd(args, trial_dir, trial)
    log_path = os.path.join(trial_dir, "roslaunch.log")

    print("start launch:", " ".join(cmd), flush=True)
    with open(log_path, "w") as log_file:
        proc = subprocess.Popen(
            cmd,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )

    time.sleep(args.startup_wait_sec)
    if proc.poll() is not None:
        raise RuntimeError("roslaunch exited early. Check log: %s" % log_path)

    return proc


def stop_launch(proc, timeout_sec):
    if proc is None or proc.poll() is not None:
        return

    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=timeout_sec)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        try:
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            proc.wait(timeout=3.0)
    except ProcessLookupError:
        pass


def cleanup_commands(args):
    for command in args.cleanup_command:
        subprocess.call(command, shell=True)


def write_header_if_needed(path, fieldnames):
    if os.path.exists(path) and os.path.getsize(path) > 0:
        return
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()


def failed_row(trial):
    return {
        "trial": trial,
        "status": "failed",
        "travel_time_sec": -1.0,
        "path_length": 0.0,
        "avg_speed": 0.0,
        "global_plan_time_mean": -1.0,
        "global_plan_time_max": -1.0,
        "global_plan_call_count": 0,
    }


def run_worker(args):
    evaluator = SameGoalEvaluator(args)
    evaluator.wait_ready()
    row = evaluator.run_trial(args.trial)
    print(RESULT_PREFIX + json.dumps(row, sort_keys=True), flush=True)
    return 0


def run_parent(args, original_argv):
    args.output_dir = os.path.join(os.path.expanduser(args.output_dir),args.method,args.env)
    os.makedirs(args.output_dir, exist_ok=True)

    summary_path = os.path.join(args.output_dir, args.summary_name)
    fieldnames = [
        "trial",
        "status",
        "travel_time_sec",
        "path_length",
        "avg_speed",
        "global_plan_time_mean",
        "global_plan_time_max",
        "global_plan_call_count",
    ]
    write_header_if_needed(summary_path, fieldnames)

    script_path = os.path.abspath(__file__)

    with open(summary_path, "a", newline="") as summary_file:
        writer = csv.DictWriter(summary_file, fieldnames=fieldnames)

        for trial in range(args.n_trials):
            trial_dir = os.path.join(args.output_dir, "trial_%03d" % trial)
            os.makedirs(trial_dir, exist_ok=True)

            launch_proc = None
            row = None

            try:
                print("[trial %03d] start" % trial, flush=True)
                launch_proc = start_launch(args, trial_dir, trial=trial)

                worker_cmd = [sys.executable, script_path, "--worker", "--trial", str(trial)] + original_argv
                worker_log_path = os.path.join(trial_dir, "worker.log")

                proc = subprocess.Popen(
                    worker_cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                )
                output, _ = proc.communicate()

                with open(worker_log_path, "w") as worker_log:
                    worker_log.write(output or "")

                for line in (output or "").splitlines():
                    if line.startswith(RESULT_PREFIX):
                        row = json.loads(line[len(RESULT_PREFIX):])

                if row is None:
                    row = failed_row(trial)
                    print("[trial %03d] worker failed. Check %s" % (trial, worker_log_path), flush=True)

            except Exception as exc:
                row = failed_row(trial)
                error_path = os.path.join(trial_dir, "error.log")
                with open(error_path, "w") as f:
                    f.write(str(exc) + "\n")
                print("[trial %03d] failed: %s" % (trial, exc), flush=True)

            finally:
                stop_launch(launch_proc, args.shutdown_wait_sec)
                cleanup_commands(args)

            writer.writerow(row)
            summary_file.flush()

            print(
                "[trial %03d] %s travel=%.3f path=%.3f avg_speed=%.3f plan_mean=%.6f plan_max=%.6f plan_count=%d" % (
                    row["trial"],
                    row["status"],
                    row["travel_time_sec"],
                    row["path_length"],
                    row["avg_speed"],
                    row["global_plan_time_mean"],
                    row["global_plan_time_max"],
                    row["global_plan_call_count"],
                ),
                flush=True,
            )

            time.sleep(args.trial_gap_sec)

    print("summary saved:", summary_path, flush=True)
    return 0


def build_arg_parser():
    parser = argparse.ArgumentParser(description="Navigation evaluator")

    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--trial", type=int, default=0, help=argparse.SUPPRESS)

    parser.add_argument("--launch-package", default="scope_nav")
    parser.add_argument("--launch-file", default="so_scope_test3.launch")
    parser.add_argument("--launch-arg", dest="launch_args", action="append", default=[])
    parser.add_argument("--startup-wait-sec", type=float, default=10.0)
    parser.add_argument("--shutdown-wait-sec", type=float, default=8.0)

    parser.add_argument("--gazebo-recording", dest="gazebo_recording", action="store_true", default=True)
    parser.add_argument("--gazebo-record-dir", default="gazebo_log")

    parser.add_argument("--n-trials", type=int, default=50)
    parser.add_argument("--timeout-sec", type=float, default=50.0)
    parser.add_argument("--goal-tolerance", type=float, default=0.9)
    parser.add_argument("--frame-id", default="odom")
    parser.add_argument("--goal-x", type=float, default=10.0)
    parser.add_argument("--goal-y", type=float, default=2.5)
    parser.add_argument("--goal-yaw", type=float, default=0.0)

    parser.add_argument("--output-dir", default="/home/glab/scope_eval")
    parser.add_argument("--method", default="so_scope_dwa")
    parser.add_argument("--env", default="test5")
    parser.add_argument("--summary-name", default="summary.csv")
    parser.add_argument("--bag-name-format", default="trial_%03d.bag")

    parser.add_argument("--move-base-action", default="/move_base")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--global-plan-time-topic", default="/move_base/NavfnROS/planning_time_ms")
    parser.add_argument("--collision-topic", default="/mobile_base/events/bumper", help="Empty string disables collision checking.")

    parser.add_argument("--check-rate", type=float, default=10.0)
    parser.add_argument("--max-odom-step", type=float, default=1.0)
    parser.add_argument("--trial-gap-sec", type=float, default=7.0)

    parser.add_argument("--use-sim-time", dest="use_sim_time", action="store_true", default=True)

    parser.add_argument("--cleanup-command", action="append", default=[
        "pkill -f 'gzclient' || true",
        "pkill -f 'gzserver' || true",
        "pkill -f 'roslaunch' || true",
        "pkill -f 'rosmaster' || true",
        "pkill -f 'rosout' || true",
    ])

    return parser


def main():
    parser = build_arg_parser()
    args = parser.parse_args()
    args.launch_args = parse_launch_args(args.launch_args)

    if args.worker:
        return run_worker(args)

    original_argv = [arg for arg in sys.argv[1:] if arg != "--worker"]
    return run_parent(args, original_argv)


if __name__ == "__main__":
    sys.exit(main())