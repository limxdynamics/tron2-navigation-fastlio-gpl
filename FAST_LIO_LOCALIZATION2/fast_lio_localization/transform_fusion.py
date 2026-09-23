#!/usr/bin/env python3

import copy
import threading
import time
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy.timer
import transforms3d.quaternions as quat_ops
import transforms3d.affines as affines
import tf2_ros
from geometry_msgs.msg import Transform
from std_msgs.msg import Header


class TransformFusion(Node):
    def __init__(self):
        super().__init__("transform_fusion")

        # 声明参数：z坐标偏移（用于调整定位高度）
        self.declare_parameter("z_offset", 0.0)
        self.z_offset = self.get_parameter("z_offset").value
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("child_frame_id", "body")
        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("map_to_odom_topic", "/map_to_odom")
        self.declare_parameter("initial_pose_topic", "/initialpose")
        self.declare_parameter("localization_topic", "/localization")
        self.declare_parameter("pose_topic", "/pose_stamped")
        self.declare_parameter("publish_before_localization", False)
        self.declare_parameter("base_yaw_offset_deg", 0.0)
        self.frame_id = self.get_parameter("frame_id").value.lstrip("/") or "map"
        self.child_frame_id = self.get_parameter("child_frame_id").value.lstrip("/") or "body"
        self.odom_topic = self.get_parameter("odom_topic").value
        self.map_to_odom_topic = self.get_parameter("map_to_odom_topic").value
        self.initial_pose_topic = self.get_parameter("initial_pose_topic").value
        self.localization_topic = self.get_parameter("localization_topic").value
        self.pose_topic = self.get_parameter("pose_topic").value
        self.publish_before_localization = self.get_parameter("publish_before_localization").value
        self.base_yaw_offset_deg = float(
            self.get_parameter("base_yaw_offset_deg").value
        )
        base_yaw_offset = np.deg2rad(self.base_yaw_offset_deg)
        self.T_lio_to_base = np.eye(4)
        self.T_lio_to_base[:3, :3] = np.array(
            [
                [np.cos(base_yaw_offset), -np.sin(base_yaw_offset), 0.0],
                [np.sin(base_yaw_offset), np.cos(base_yaw_offset), 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        self.get_logger().info(f"Z offset parameter: {self.z_offset} m")
        self.get_logger().info(
            f"FAST-LIO-to-base yaw offset: {self.base_yaw_offset_deg:.1f} deg"
        )

        self.cur_odom_to_baselink = None
        self.cur_map_to_odom = None

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.pub_localization = self.create_publisher(Odometry, self.localization_topic, 1)
        self.pub_pose = self.create_publisher(PoseStamped, self.pose_topic, 1)

        self.create_subscription(Odometry, self.odom_topic, self.cb_save_cur_odom, 1)
        self.create_subscription(Odometry, self.map_to_odom_topic, self.cb_save_map_to_odom, 1)
        self.create_subscription(
            PoseWithCovarianceStamped, self.initial_pose_topic, self.cb_reset_localization, 1
        )

        self.freq_pub_localization = 50
        self.timer = self.create_timer(1/self.freq_pub_localization, self.transform_fusion)
        # threading.Thread(target=self.transform_fusion, daemon=True).start()

    def pose_to_mat(self, pose_msg):
        trans = np.eye(4)
        trans[:3, 3] = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
        # transforms3d uses [w, x, y, z] order, while ROS uses [x, y, z, w]
        quat = [pose_msg.orientation.w, pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z]
        trans[:3, :3] = quat_ops.quat2mat(quat)
        return trans

    def transform_fusion(self):
        if self.cur_odom_to_baselink is None:
            return

        if self.cur_map_to_odom is not None:
            T_map_to_odom = self.pose_to_mat(self.cur_map_to_odom.pose.pose)
            # 打印变换信息用于调试
            if not hasattr(self, '_last_logged_transform') or \
               np.linalg.norm(T_map_to_odom[:3, 3] - self._last_logged_transform[:3, 3]) > 0.1:
                self.get_logger().info(
                    f"📍 map->odom transform: translation=({T_map_to_odom[0,3]:.2f}, {T_map_to_odom[1,3]:.2f}, {T_map_to_odom[2,3]:.2f})"
                )
                self._last_logged_transform = T_map_to_odom.copy()
        else:
            if not self.publish_before_localization:
                if not hasattr(self, '_warned_no_map_to_odom'):
                    self.get_logger().warn(
                        "No map_to_odom transform received; navigation pose output is gated."
                    )
                    self._warned_no_map_to_odom = True
                return
            T_map_to_odom = np.eye(4)
            if not hasattr(self, '_warned_no_map_to_odom'):
                self.get_logger().warn(
                    "⚠️  No map_to_odom transform received! Using identity transform. "
                    "This means map and odom frames are aligned (robot at map origin)."
                )
                self._warned_no_map_to_odom = True

        transform_msg = Transform()
        transform_msg.translation.x = T_map_to_odom[0, 3]
        transform_msg.translation.y = T_map_to_odom[1, 3]
        transform_msg.translation.z = T_map_to_odom[2, 3]
        
        # transforms3d returns [w, x, y, z], convert to ROS [x, y, z, w]
        quat = quat_ops.mat2quat(T_map_to_odom[:3, :3])

        transform_msg.rotation.x = quat[1]
        transform_msg.rotation.y = quat[2]
        transform_msg.rotation.z = quat[3]
        transform_msg.rotation.w = quat[0]
        
        # print(self.cur_odom_to_baselink.header)
        # 发布 map->camera_init 的 TF（保持向后兼容）
        transform_stamped_msg = tf2_ros.TransformStamped(
                header = self.cur_odom_to_baselink.header,
                child_frame_id = "camera_init",
                transform = transform_msg
            )
        transform_stamped_msg.header.frame_id = self.frame_id
        self.tf_broadcaster.sendTransform(transform_stamped_msg)

        # 同时发布 map->odom 的 TF（用于导航器路径转换）
        # 注意：如果 camera_init 和 odom 是同一个坐标系，使用相同的变换
        # 如果需要区分，可以添加参数配置
        transform_stamped_odom = tf2_ros.TransformStamped()
        transform_stamped_odom.header.stamp = self.get_clock().now().to_msg()
        transform_stamped_odom.header.frame_id = self.frame_id
        transform_stamped_odom.child_frame_id = "odom"
        transform_stamped_odom.transform = transform_msg
        self.tf_broadcaster.sendTransform(transform_stamped_odom)

        cur_odom = copy.copy(self.cur_odom_to_baselink)
        if cur_odom is not None:
            T_odom_to_base_link = self.pose_to_mat(cur_odom.pose.pose)
            T_map_to_base_link = np.matmul(
                np.matmul(T_map_to_odom, T_odom_to_base_link),
                self.T_lio_to_base,
            )

            xyz = T_map_to_base_link[:3, 3].copy()
            # 应用z坐标偏移
            xyz[2] += self.z_offset
            # transforms3d returns [w, x, y, z], convert to ROS [x, y, z, w]
            quat = quat_ops.mat2quat(T_map_to_base_link[:3, :3])

            localization = Odometry()
            localization.pose.pose = Pose(
                position = Point(x = xyz[0], y = xyz[1], z = xyz[2]), 
                orientation = Quaternion(x = quat[1], y = quat[2], z = quat[3], w = quat[0])
            )
            localization.twist = copy.deepcopy(cur_odom.twist)
            # Odometry twist is expressed in child_frame_id. Rotate the raw
            # FAST-LIO-frame vectors into the corrected robot-base frame.
            yaw = np.deg2rad(self.base_yaw_offset_deg)
            cos_yaw = np.cos(yaw)
            sin_yaw = np.sin(yaw)
            for vector in (
                localization.twist.twist.linear,
                localization.twist.twist.angular,
            ):
                old_x, old_y = vector.x, vector.y
                vector.x = cos_yaw * old_x + sin_yaw * old_y
                vector.y = -sin_yaw * old_x + cos_yaw * old_y

            localization.header.stamp = self.get_clock().now().to_msg()
            localization.header.frame_id = self.frame_id
            localization.child_frame_id = self.child_frame_id
            self.pub_localization.publish(localization)

            pose_stamped = PoseStamped()
            pose_stamped.header = localization.header
            pose_stamped.pose = localization.pose.pose
            self.pub_pose.publish(pose_stamped)


    def cb_save_cur_odom(self, msg):
        self.cur_odom_to_baselink = msg

    def cb_save_map_to_odom(self, msg):
        self.cur_map_to_odom = msg

    def cb_reset_localization(self, _msg):
        # Stop publishing the previous map pose while a new ICP initialization
        # is pending. A fresh map_to_odom message re-enables navigation output.
        self.cur_map_to_odom = None
        if hasattr(self, '_warned_no_map_to_odom'):
            del self._warned_no_map_to_odom


def main(args=None):
    rclpy.init(args=args)
    node = TransformFusion()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
