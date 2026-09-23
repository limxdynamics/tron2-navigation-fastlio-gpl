#!/usr/bin/env python3

import copy
import threading
import time

import open3d as o3d
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
# from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import tf2_ros
import transforms3d.quaternions as quat_ops


class FastLIOLocalization(Node):
    def __init__(self):
        super().__init__("fast_lio_localization")
        self.global_map = None
        self.T_map_to_odom = np.eye(4)
        self.localization_guess = np.eye(4)
        self.cur_odom = None
        self.cur_scan = None
        self.initialized = False
        self.localized = False
        self.pending_initial_base_pose = None

        self.declare_parameters(
            namespace="",
            parameters=[
                ("map_voxel_size", 0.4),
                ("scan_voxel_size", 0.1),
                ("freq_localization", 0.5),
                ("freq_global_map", 0.25),
                ("localization_threshold", 0.8),
                ("fov", 6.28319),
                ("fov_far", 300),
                ("pcd_map_topic", "/map"),
                ("pcd_map_path", ""),
                ("frame_id", "map"),
                ("scan_topic", "/cloud_registered"),
                ("odom_topic", "/Odometry"),
                ("initial_pose_topic", "/initialpose"),
                ("map_to_odom_topic", "/map_to_odom"),
                ("corrected_cloud_topic", "/corrected_current_pcd"),
                ("legacy_corrected_cloud_topic", "/cur_scan_in_map"),
                ("base_yaw_offset_deg", 0.0),
            ],
        )

        self.frame_id = self.get_parameter("frame_id").value.lstrip("/") or "map"
        self.scan_topic = self.get_parameter("scan_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.initial_pose_topic = self.get_parameter("initial_pose_topic").value
        self.map_to_odom_topic = self.get_parameter("map_to_odom_topic").value
        self.corrected_cloud_topic = self.get_parameter("corrected_cloud_topic").value
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
        legacy_cloud_topic = self.get_parameter("legacy_corrected_cloud_topic").value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # self.pub_global_map = self.create_publisher(PointCloud2, self.get_parameter("pcd_map_topic").value, 10)
        self.pub_pc_in_map = self.create_publisher(PointCloud2, self.corrected_cloud_topic, 10)
        self.pub_legacy_pc_in_map = None
        if legacy_cloud_topic and legacy_cloud_topic != self.corrected_cloud_topic:
            self.pub_legacy_pc_in_map = self.create_publisher(PointCloud2, legacy_cloud_topic, 10)
        self.pub_submap = self.create_publisher(PointCloud2, "/submap", 10)
        self.pub_map_to_odom = self.create_publisher(Odometry, self.map_to_odom_topic, 10)

        self.get_logger().info("Waiting for global map...")
        # global_map_msg = wait_for_message(msg_type = PointCloud2, node = self, topic = "/cloud_pcd")[1]
        # self.initialize_global_map(global_map_msg)
        
        self.initialize_global_map()
        self.get_logger().info("Global map received.")
        
        self.create_subscription(PointCloud2, self.scan_topic, self.cb_save_cur_scan, 10)
        self.create_subscription(Odometry, self.odom_topic, self.cb_save_cur_odom, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, self.initial_pose_topic, self.cb_initialize_pose, 10
        )

        self.get_logger().info(
            f"Navigation outputs: pose transform={self.map_to_odom_topic}, "
            f"corrected cloud={self.corrected_cloud_topic}, frame={self.frame_id}, "
            f"base yaw offset={self.base_yaw_offset_deg:.1f} deg"
        )

        self.timer_localisation = self.create_timer(1.0 / self.get_parameter("freq_localization").value, self.localisation_timer_callback)
        # self.timer_global_map = self.create_timer(1/ self.get_parameter("freq_global_map").value, self.global_map_callback)

    def global_map_callback(self):
        # self.get_logger().info(np.array(self.global_map.points).shape)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        self.publish_point_cloud(self.pub_global_map, header, np.array(self.global_map.points))
        
    def pose_to_mat(self, pose):
        trans = np.eye(4)
        trans[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        # transforms3d uses [w, x, y, z] order, while ROS uses [x, y, z, w]
        quat = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
        trans[:3, :3] = quat_ops.quat2mat(quat)
        return trans
    
    def msg_to_array(self, pc_msg):
        if pc_msg.height <= 0 or pc_msg.width <= 0 or pc_msg.point_step <= 0:
            raise ValueError("PointCloud2 has invalid dimensions or point_step")

        fields = {field.name: field for field in pc_msg.fields}
        missing = {"x", "y", "z"}.difference(fields)
        if missing:
            raise ValueError(
                f"PointCloud2 is missing fields: {sorted(missing)}"
            )
        for name in ("x", "y", "z"):
            field = fields[name]
            if field.datatype != PointField.FLOAT32 or field.count != 1:
                raise ValueError(f"PointCloud2 field '{name}' must be FLOAT32")
            if field.offset < 0 or field.offset + 4 > pc_msg.point_step:
                raise ValueError(f"PointCloud2 field '{name}' has an invalid offset")

        packed_row_bytes = int(pc_msg.width) * int(pc_msg.point_step)
        if pc_msg.row_step < packed_row_bytes:
            raise ValueError("PointCloud2 row_step is smaller than its packed row")
        expected_bytes = int(pc_msg.row_step) * int(pc_msg.height)
        raw = np.asarray(pc_msg.data, dtype=np.uint8)
        if raw.nbytes < expected_bytes:
            raise ValueError(
                f"PointCloud2 data is truncated: {raw.nbytes} < {expected_bytes}"
            )

        byte_order = ">" if pc_msg.is_bigendian else "<"
        dtype = np.dtype(
            {
                "names": ("x", "y", "z"),
                "formats": (byte_order + "f4",) * 3,
                "offsets": tuple(fields[name].offset for name in ("x", "y", "z")),
                "itemsize": int(pc_msg.point_step),
            }
        )
        rows = []
        for row in range(int(pc_msg.height)):
            start = row * int(pc_msg.row_step)
            row_data = raw[start : start + packed_row_bytes]
            records = np.frombuffer(
                row_data, dtype=dtype, count=int(pc_msg.width)
            )
            rows.append(
                np.column_stack(
                    (records["x"], records["y"], records["z"])
                )
            )
        points = np.concatenate(rows, axis=0).astype(np.float32, copy=False)
        return np.ascontiguousarray(points[np.isfinite(points).all(axis=1)])

    def transform_points(self, points, transform):
        points = np.asarray(points)
        return points[:, :3] @ transform[:3, :3].T + transform[:3, 3]
    
    def registration_at_scale(self, scan, map, initial, scale):
        result_icp = o3d.pipelines.registration.registration_icp(
        self.voxel_down_sample(scan, self.get_parameter("scan_voxel_size").value * scale),
        self.voxel_down_sample(map, self.get_parameter("map_voxel_size").value * scale),
        1.0 * scale,
        initial,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20),
        )
        return result_icp.transformation, result_icp.fitness
            
    def inverse_se3(self, trans):
        trans_inverse = np.eye(4)
        # R
        trans_inverse[:3, :3] = trans[:3, :3].T
        # t
        trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
        return trans_inverse

    def initial_base_pose_to_map_to_odom(self, initial_base_pose):
        # RViz specifies the current robot-base pose in map. FAST-LIO odometry
        # specifies the current LiDAR/IMU pose in odom, so solve:
        #   T_map_odom = T_map_base * inverse(T_odom_lio * T_lio_base)
        odom_to_lio = self.pose_to_mat(self.cur_odom.pose.pose)
        odom_to_base = np.matmul(odom_to_lio, self.T_lio_to_base)
        return np.matmul(initial_base_pose, self.inverse_se3(odom_to_base))

    def publish_point_cloud(self, publisher, header, pc):
        points = np.ascontiguousarray(pc[:, :3], dtype="<f4")
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.data = points.tobytes()
        msg.is_dense = bool(np.isfinite(points).all())
        publisher.publish(msg)
        
    def crop_global_map_in_FOV(self, pose_estimation):
        T_odom_to_base_link = self.pose_to_mat(self.cur_odom.pose.pose)
        T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
        T_base_link_to_map = self.inverse_se3(T_map_to_base_link)

        global_map_in_map = np.array(self.global_map.points)
        global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
        global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T

        if self.get_parameter("fov").value > 3.14:
            indices = np.where(
                (global_map_in_base_link[:, 0] < self.get_parameter("fov_far").value)
                & (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < self.get_parameter("fov").value / 2.0)
            )
        else:
            indices = np.where(
                (global_map_in_base_link[:, 0] > 0)
                & (global_map_in_base_link[:, 0] < self.get_parameter("fov_far").value)
                & (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < self.get_parameter("fov").value / 2.0)
            )
        selected_points = global_map_in_map[indices[0], :3].reshape(-1, 3)
        global_map_in_FOV = o3d.geometry.PointCloud()
        global_map_in_FOV.points = o3d.utility.Vector3dVector(selected_points)

        header = copy.copy(self.cur_odom.header)
        header.frame_id = self.frame_id
        self.publish_point_cloud(self.pub_submap, header, np.array(global_map_in_FOV.points)[::10])

        return global_map_in_FOV

    def global_localization(self, pose_estimation):
        if self.cur_scan is None or self.cur_odom is None:
            return

        scan_tobe_mapped = copy.copy(self.cur_scan)
        global_map_in_FOV = self.crop_global_map_in_FOV(pose_estimation)

        if len(scan_tobe_mapped.points) == 0 or len(global_map_in_FOV.points) == 0:
            self.get_logger().warn("Skipping ICP because the scan or local map is empty.")
            return
        
        self.get_logger().info(f"Performing ICP registration: scan points={len(scan_tobe_mapped.points)}, map points={len(global_map_in_FOV.points)}")
        
        try:
            transformation, _ = self.registration_at_scale(
                scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5
            )
            transformation, fitness = self.registration_at_scale(
                scan_tobe_mapped, global_map_in_FOV, initial=transformation, scale=1
            )
        except (RuntimeError, ValueError) as error:
            self.get_logger().error(f"ICP registration failed: {error}")
            return

        self.get_logger().info(f"ICP registration completed. Fitness score: {fitness:.4f}, Threshold: {self.get_parameter('localization_threshold').value}")
        
        if fitness > self.get_parameter("localization_threshold").value:
            self.T_map_to_odom = transformation
            self.localization_guess = transformation
            self.localized = True
            self.publish_odom(transformation)
            self.get_logger().info(f"Localization successful! Published map_to_odom transform.")
        else:
            self.get_logger().warn(f"Fitness score {fitness:.4f} less than localization threshold {self.get_parameter('localization_threshold').value}")

    def voxel_down_sample(self, pcd, voxel_size):
        # print(pcd)
        
        try:
            pcd_down = pcd.voxel_down_sample(voxel_size)
        
        except Exception as e:
            # for opend3d 0.7 or lower
            pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)
            
        return pcd_down

    def cb_save_cur_odom(self, msg):
        self.cur_odom = msg
        
    def cb_save_cur_scan(self, msg):
        try:
            pc = self.msg_to_array(msg)
        except ValueError as error:
            self.get_logger().error(f"Ignoring malformed point cloud: {error}")
            return
        self.cur_scan = o3d.geometry.PointCloud()
        self.cur_scan.points = o3d.utility.Vector3dVector(pc)
        if len(pc) > 0:
            self.get_logger().debug(f"Received scan with {len(pc)} points")

        # /cloud_registered is expressed in FAST-LIO's local camera_init frame.
        # SCAN requires obstacle points in the fixed map frame, so publish only
        # after ICP has established the map <- camera_init transform.
        if self.localized and len(pc) > 0:
            corrected_pc = self.transform_points(pc, self.T_map_to_odom)
            header = copy.copy(msg.header)
            header.frame_id = self.frame_id
            self.publish_point_cloud(self.pub_pc_in_map, header, corrected_pc)
            if self.pub_legacy_pc_in_map is not None:
                self.publish_point_cloud(self.pub_legacy_pc_in_map, header, corrected_pc)
        
    def initialize_global_map(self): #, pc_msg):
        # self.global_map = o3d.geometry.PointCloud()
        # self.global_map.points = o3d.utility.Vector3dVector(self.msg_to_array(pc_msg)[:, :3])
        map_path = self.get_parameter("pcd_map_path").value
        if not map_path:
            raise RuntimeError("pcd_map_path is empty")
        self.global_map = o3d.io.read_point_cloud(map_path)
        if len(self.global_map.points) == 0:
            raise RuntimeError(f"PCD map is empty or unreadable: {map_path}")
        self.global_map = self.voxel_down_sample(self.global_map, self.get_parameter("map_voxel_size").value)
        self.get_logger().info("Global map received.")

    def cb_initialize_pose(self, msg):
        msg_frame = msg.header.frame_id.lstrip("/")
        if msg_frame and msg_frame != self.frame_id:
            self.get_logger().warn(
                f"Ignoring initial pose in frame '{msg.header.frame_id}'; expected '{self.frame_id}'."
            )
            return

        # RViz initialpose describes the robot base at its current odometry
        # pose. Keep it pending until both scan and odometry are available.
        initial_base_pose = self.pose_to_mat(msg.pose.pose)
        self.pending_initial_base_pose = initial_base_pose
        self.initialized = True
        self.localized = False
        self.get_logger().info("Initial pose received.")
        
        if self.cur_scan is not None and self.cur_odom is not None:
            initial_pose = self.initial_base_pose_to_map_to_odom(
                self.pending_initial_base_pose
            )
            self.pending_initial_base_pose = None
            self.localization_guess = initial_pose
            self.get_logger().info("Starting global localization with current scan...")
            self.global_localization(initial_pose)
        else:
            self.get_logger().warn(
                f"Waiting for {self.scan_topic} and {self.odom_topic} before localization."
            )
            
    def publish_odom(self, transform):
        odom_msg = Odometry()
        xyz = transform[:3, 3]
        # transforms3d returns [w, x, y, z], convert to ROS [x, y, z, w]
        quat = quat_ops.mat2quat(transform[:3, :3])
        odom_msg.pose.pose = Pose(
            position = Point(x = xyz[0], y = xyz[1], z = xyz[2]), 
            orientation = Quaternion(x = quat[1], y = quat[2], z = quat[3], w = quat[0])
        )
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.frame_id
        self.pub_map_to_odom.publish(odom_msg)

    def localisation_timer_callback(self):
        if not self.initialized:
            self.get_logger().info("Waiting for initial pose...")
            return
        
        if self.cur_scan is not None and self.cur_odom is not None:
            if self.pending_initial_base_pose is not None:
                pose_estimation = self.initial_base_pose_to_map_to_odom(
                    self.pending_initial_base_pose
                )
                self.pending_initial_base_pose = None
                self.localization_guess = pose_estimation
            else:
                pose_estimation = (
                    self.T_map_to_odom if self.localized else self.localization_guess
                )
            self.global_localization(pose_estimation)
        elif self.cur_scan is None:
            self.get_logger().warn(f"Waiting for {self.scan_topic} topic data...")
        elif self.cur_odom is None:
            self.get_logger().warn(f"Waiting for {self.odom_topic} topic data...")


def main(args=None):
    rclpy.init(args=args)
    node = FastLIOLocalization()
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