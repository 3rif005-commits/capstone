#!/usr/bin/env python3
"""Debug: does SetEntityPose actually move the cine_cam sensor view?

Sets the cine_cam to three very different poses and saves a frame at each
(/tmp/cam_0.png, cam_1.png, cam_2.png). If the three images are identical, the
camera sensor render is NOT following set_pose. Requires gz + the service bridge
+ the image bridge to be running.
"""
import time
import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ros_gz_interfaces.srv import SetEntityPose

from vtol_sim.machinima.camera_moves import look_at_quat

POSES = [
    ((45, -25, 6),  (45, 0, 1)),    # side view of the tank
    ((5,   0, 45),  (45, 0, 1)),    # high oblique from the west
    ((45,  0, 70),  (45, 0, 0)),    # straight top-down
]


def main():
    rclpy.init()
    node = rclpy.create_node('debug_camera')
    bridge = CvBridge()
    latest = {'img': None}
    node.create_subscription(
        Image, '/cine_cam/image',
        lambda m: latest.__setitem__('img', bridge.imgmsg_to_cv2(m, 'bgr8')), 10)

    cli = node.create_client(SetEntityPose, '/world/machinima_world/set_pose')
    cli.wait_for_service(timeout_sec=10.0)

    for i, (eye, tgt) in enumerate(POSES):
        qx, qy, qz, qw = look_at_quat(eye, tgt)
        req = SetEntityPose.Request()
        req.entity.name = 'cine_cam'
        req.entity.type = 2
        req.pose = Pose()
        req.pose.position.x, req.pose.position.y, req.pose.position.z = map(float, eye)
        req.pose.orientation.x = qx
        req.pose.orientation.y = qy
        req.pose.orientation.z = qz
        req.pose.orientation.w = qw
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=3.0)
        ok = fut.result().success if fut.done() and fut.result() else '??'
        print(f'[dbg] pose {i} set_pose success={ok} eye={eye}')

        # let it move + grab a fresh frame
        latest['img'] = None
        t0 = time.time()
        while time.time() - t0 < 2.5:
            rclpy.spin_once(node, timeout_sec=0.1)
        if latest['img'] is not None:
            cv2.imwrite(f'/tmp/cam_{i}.png', latest['img'])
            print(f'[dbg] saved /tmp/cam_{i}.png mean={latest["img"].mean():.1f}')
        else:
            print(f'[dbg] NO FRAME for pose {i}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
