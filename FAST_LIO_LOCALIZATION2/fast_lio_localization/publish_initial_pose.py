#!/usr/bin/env python3

import argparse
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped


class PublishInitialPose(Node):
    def __init__(self):
        super().__init__("publish_initial_pose")
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

    def publish_pose(self, x, y, z, roll, pitch, yaw):
        # Static-axis XYZ roll/pitch/yaw to ROS quaternion [x, y, z, w].
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        quat = (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.pose.pose = Pose(
            position=Point(x=x, y=y, z=z),
            orientation=Quaternion(
                x=quat[0], y=quat[1], z=quat[2], w=quat[3]
            ),
        )
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = "map"
        self.pub_pose.publish(initial_pose)


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("z", type=float)
    parser.add_argument("yaw", type=float)
    parser.add_argument("pitch", type=float)
    parser.add_argument("roll", type=float)
    pose_args, ros_args = parser.parse_known_args(args=args)
    values = (
        pose_args.x,
        pose_args.y,
        pose_args.z,
        pose_args.yaw,
        pose_args.pitch,
        pose_args.roll,
    )
    if not all(math.isfinite(value) for value in values):
        parser.error("pose values must be finite")

    rclpy.init(args=ros_args)
    node = PublishInitialPose()
    try:
        deadline = time.monotonic() + 5.0
        while (
            rclpy.ok()
            and node.pub_pose.get_subscription_count() == 0
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.1)

        if node.pub_pose.get_subscription_count() == 0:
            node.get_logger().error(
                "No /initialpose subscriber discovered within 5 seconds."
            )
            return 1

        for _ in range(3):
            node.publish_pose(
                pose_args.x,
                pose_args.y,
                pose_args.z,
                pose_args.roll,
                pose_args.pitch,
                pose_args.yaw,
            )
            rclpy.spin_once(node, timeout_sec=0.1)
        node.get_logger().info(
            "Initial base pose published in map: "
            f"xyz=({pose_args.x}, {pose_args.y}, {pose_args.z}), "
            f"rpy=({pose_args.roll}, {pose_args.pitch}, {pose_args.yaw})"
        )
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
