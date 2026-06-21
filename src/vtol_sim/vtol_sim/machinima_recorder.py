#!/usr/bin/env python3
"""Machinima recorder — cine-cam image stream -> mp4.

Subscribes to the cinematic camera's image topic (bridged from Gazebo by
``ros_gz_image``) and writes frames to an mp4 via ``cv2.VideoWriter``. Recording
is gated by a latched ``std_msgs/Bool`` on ``/machinima/record`` so the director
can start/stop clean takes:

  * ``True``  -> open a new timestamped file and start writing frames
  * ``False`` -> close the current file

Output lands in ``media/`` under the package working dir (override with the
``out_dir`` param). FPS is fixed by the ``fps`` param and must match the cine-cam
sensor ``update_rate`` for real-time playback.

Run (after the launch file is up):
  ros2 run vtol_sim machinima_recorder
"""
import os
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class MachinimaRecorder(Node):
    def __init__(self):
        super().__init__('machinima_recorder')
        self.declare_parameter('image_topic', '/cine_cam/image')
        self.declare_parameter('out_dir', 'media')
        self.declare_parameter('fps', 30.0)

        self._topic = self.get_parameter('image_topic').value
        self._out_dir = self.get_parameter('out_dir').value
        self._fps = float(self.get_parameter('fps').value)

        os.makedirs(self._out_dir, exist_ok=True)
        self._bridge = CvBridge()
        self._writer = None
        self._path = None
        self._frames = 0
        self._t_first = None
        self._t_last = None
        self._size = None

        self.create_subscription(Image, self._topic, self._on_image, 10)

        latched = QoSProfile(depth=1,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, '/machinima/record', self._on_record, latched)

        print(f'[REC] ready — recording {self._topic} @ {self._fps:g} fps '
              f'into {self._out_dir}/ when /machinima/record = true')

    # ── Record gate ───────────────────────────────────────────────────────────
    def _on_record(self, msg: Bool):
        if msg.data and self._writer is None:
            self._open()
        elif not msg.data and self._writer is not None:
            self._close()

    def _open(self):
        stamp = time.strftime('%Y%m%d_%H%M%S')
        self._path = os.path.join(self._out_dir, f'machinima_{stamp}.mp4')
        self._frames = 0
        self._t_first = None
        self._t_last = None
        self._size = None
        # Writer is opened lazily on the first frame (needs the frame size).
        print(f'[REC] ARMED -> {self._path}')

    def _close(self):
        if self._writer is not None:
            self._writer.release()
            secs = self._frames / self._fps if self._fps else 0.0
            print(f'[REC] STOP  -> {self._path}  ({self._frames} frames, '
                  f'{secs:.1f}s @ {self._fps:g}fps)')
            self._retime()                      # rewrite to real-time playback
        self._writer = None
        self._path = None

    def _retime(self):
        """Headless/loaded renders rarely hit the target fps, so the raw clip
        plays fast/slow. Re-encode it at the *measured* capture rate so playback
        duration matches real time. Uses cv2 only (no ffmpeg)."""
        if (self._t_first is None or self._t_last is None
                or self._frames < 2 or self._size is None):
            return
        wall = self._t_last - self._t_first
        if wall <= 0.1:
            return
        real_fps = (self._frames - 1) / wall
        if abs(real_fps - self._fps) < 0.5:
            return                              # already close enough
        out = self._path.replace('.mp4', '_realtime.mp4')
        cap = cv2.VideoCapture(self._path)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        wr = cv2.VideoWriter(out, fourcc, real_fps, self._size)
        n = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            wr.write(frame)
            n += 1
        cap.release()
        wr.release()
        print(f'[REC] RETIMED -> {out}  ({n} frames @ {real_fps:.1f}fps, '
              f'{n / real_fps:.1f}s real time)')

    # ── Frames ────────────────────────────────────────────────────────────────
    def _on_image(self, msg: Image):
        if self._path is None:
            return                              # not recording
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        now = time.monotonic()
        if self._writer is None:
            h, w = frame.shape[:2]
            self._size = (w, h)
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self._writer = cv2.VideoWriter(self._path, fourcc, self._fps, (w, h))
            self._t_first = now
            print(f'[REC] REC   -> {self._path}  ({w}x{h})')
        self._writer.write(frame)
        self._t_last = now
        self._frames += 1

    def destroy_node(self):
        self._close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MachinimaRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
