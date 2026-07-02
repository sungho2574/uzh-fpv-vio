#!/usr/bin/env python
"""
Record two videos while a VINS-Mono variant runs (ROS Kinetic / Python 2.7):

  1. vio_track.mp4     -- feature_tracker's annotated /feature_tracker/feature_img
                           stream (raw feature points on the camera image), upscaled.
  2. vio_track_3d.mp4   -- the same feed, with a live 3D plot (estimated trajectory +
                           a 3-axis attitude triad at the current pose, from
                           /vins_estimator/odometry) composited into the bottom-right
                           corner.

Meant to be started in the background by run_vins.sh right before `rosbag play`,
and stopped with SIGINT/SIGTERM once playback + the flush window are done. On
shutdown it flushes and closes both video files.

Usage:
    python2 record_video.py --out-dir /root/output/<variant> [--fps 10] [--scale 2.0] [--inset-scale 0.42]
"""
from __future__ import division, print_function

import argparse
import io
import os
import signal
import threading

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 -- registers the '3d' projection

FEATURE_IMG_TOPIC = "/feature_tracker/feature_img"
ODOMETRY_TOPIC = "/vins_estimator/odometry"


def quat_to_axis_vectors(qx, qy, qz, qw, length):
    """Rotate the unit x/y/z axes by the given quaternion, scaled to `length`."""
    R = np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])
    return R[:, 0] * length, R[:, 1] * length, R[:, 2] * length


class VideoRecorder(object):
    def __init__(self, out_dir, fps, scale, inset_scale):
        self.bridge = CvBridge()
        self.out_dir = out_dir
        self.fps = fps
        self.scale = scale
        self.inset_scale = inset_scale

        self.writer_plain = None
        self.writer_combo = None

        self.pose_lock = threading.Lock()
        self.traj = []
        self.latest_pos = (0.0, 0.0, 0.0)
        self.latest_quat = (0.0, 0.0, 0.0, 1.0)  # x, y, z, w
        self.have_pose = False

        self.fig = plt.figure(figsize=(4, 4), dpi=100)
        self.ax = self.fig.add_subplot(111, projection="3d")

        self.odom_sub = rospy.Subscriber(ODOMETRY_TOPIC, Odometry, self._odom_cb, queue_size=50)
        self.img_sub = rospy.Subscriber(FEATURE_IMG_TOPIC, Image, self._img_cb, queue_size=10)

        self.n_frames = 0

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        with self.pose_lock:
            self.latest_pos = (p.x, p.y, p.z)
            self.latest_quat = (q.x, q.y, q.z, q.w)
            self.have_pose = True

    def _render_3d_plot(self):
        with self.pose_lock:
            pos = self.latest_pos
            quat = self.latest_quat
            traj = list(self.traj)

        ax = self.ax
        ax.clear()

        if len(traj) >= 2:
            xs, ys, zs = zip(*traj)
            ax.plot(xs, ys, zs, color="tab:blue", linewidth=1.5)
            span = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs), 1e-3)
            cx = (max(xs) + min(xs)) / 2.0
            cy = (max(ys) + min(ys)) / 2.0
            cz = (max(zs) + min(zs)) / 2.0
            half = span / 2.0 * 1.3 + 0.3
            ax.set_xlim(cx - half, cx + half)
            ax.set_ylim(cy - half, cy + half)
            ax.set_zlim(cz - half, cz + half)
            axis_len = span * 0.12 + 0.05
        else:
            axis_len = 0.3
            ax.set_xlim(-1, 1)
            ax.set_ylim(-1, 1)
            ax.set_zlim(-1, 1)

        px, py, pz = pos
        xa, ya, za = quat_to_axis_vectors(quat[0], quat[1], quat[2], quat[3], axis_len)
        ax.quiver(px, py, pz, xa[0], xa[1], xa[2], color="r", linewidth=2)
        ax.quiver(px, py, pz, ya[0], ya[1], ya[2], color="g", linewidth=2)
        ax.quiver(px, py, pz, za[0], za[1], za[2], color="b", linewidth=2)
        ax.scatter([px], [py], [pz], color="k", s=12)

        ax.set_xlabel("x", fontsize=7)
        ax.set_ylabel("y", fontsize=7)
        ax.set_zlabel("z", fontsize=7)
        ax.set_title("VIO trajectory + attitude", fontsize=9)
        ax.tick_params(labelsize=6)

        buf = io.BytesIO()
        self.fig.savefig(buf, format="png", dpi=100)
        buf.seek(0)
        arr = np.frombuffer(buf.read(), dtype=np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)

    def _img_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = frame.shape[:2]
        if self.scale != 1.0:
            frame = cv2.resize(frame, (int(w * self.scale), int(h * self.scale)))
            h, w = frame.shape[:2]

        if self.writer_plain is None:
            if not os.path.isdir(self.out_dir):
                os.makedirs(self.out_dir)
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self.writer_plain = cv2.VideoWriter(
                os.path.join(self.out_dir, "vio_track.mp4"), fourcc, self.fps, (w, h))
            self.writer_combo = cv2.VideoWriter(
                os.path.join(self.out_dir, "vio_track_3d.mp4"), fourcc, self.fps, (w, h))

        self.writer_plain.write(frame)

        if self.have_pose:
            with self.pose_lock:
                self.traj.append(self.latest_pos)

        plot_img = self._render_3d_plot()
        inset_w = int(w * self.inset_scale)
        inset_h = int(h * self.inset_scale)
        inset = cv2.resize(plot_img, (inset_w, inset_h))

        combo = frame.copy()
        y0, x0 = h - inset_h, w - inset_w
        combo[y0:h, x0:w] = inset
        self.writer_combo.write(combo)

        self.n_frames += 1

    def close(self):
        if self.writer_plain is not None:
            self.writer_plain.release()
        if self.writer_combo is not None:
            self.writer_combo.release()
        plt.close(self.fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--fps", type=float, default=10.0,
                         help="output video fps; should roughly match feature_tracker's 'freq' config")
    parser.add_argument("--scale", type=float, default=2.0,
                         help="upscale factor for the (tiny, 346x260) camera frame before writing")
    parser.add_argument("--inset-scale", type=float, default=0.42,
                         help="3D plot inset size, as a fraction of the (scaled) frame width/height")
    args, _ = parser.parse_known_args(rospy.myargv()[1:])

    rospy.init_node("video_recorder", anonymous=False, disable_signals=True)
    recorder = VideoRecorder(args.out_dir, args.fps, args.scale, args.inset_scale)

    stop = {"flag": False}

    def _handle_signal(signum, frame):
        stop["flag"] = True

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    rate = rospy.Rate(20)
    while not stop["flag"] and not rospy.is_shutdown():
        rate.sleep()

    recorder.close()
    print("video_recorder: wrote {} frames to {}".format(recorder.n_frames, args.out_dir))


if __name__ == "__main__":
    main()
