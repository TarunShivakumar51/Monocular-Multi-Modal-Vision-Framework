#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mono_perception_msgs.msg import ImageTriple
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TransformStamped, Transform, Vector3
from sensor_msgs.msg import PointCloud2, PointField
from cv_bridge import CvBridge,CvBridgeError
from mono_perception_pipeline.optical_flow import compute_optical_flow
from mono_perception_pipeline.triangulation import compute_point_cloud
import numpy as np
from scipy.spatial.transform import Rotation
from tf2_ros import TransformBroadcaster


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("image_subscriber")

        self.image_subscriber = self.create_subscription(ImageTriple, "/image_triples", self.create_path, 10)

        self.R_global = np.eye(3)
        self.t_global = np.zeros((3,1))
        self.first_message = True
        self.mtx = None

        self.path = Path()
        self.path.header.frame_id = "world"

        self.bridge = CvBridge()

        self.tf_broadcaster = TransformBroadcaster(self)

        self.path_publisher = self.create_publisher(Path, '/camera/path', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/camera/pose', 10)
        self.point_cloud_publisher = self.create_publisher(PointCloud2, '/camera/point_cloud', 10)

    def create_path(self, msg: ImageTriple):
        
        try:
            image_a = self.bridge.imgmsg_to_cv2(msg.image_a, "bgr8")
            image_b = self.bridge.imgmsg_to_cv2(msg.image_b, "bgr8")
            image_c = self.bridge.imgmsg_to_cv2(msg.image_c, "bgr8")
        except CvBridgeError as e:
            self.get_logger().info(str(e))
            return

        self.path.header.stamp = msg.header.stamp

        if self.first_message:
            first_pose = PoseStamped()

            # Header for this individual pose
            first_pose.header.frame_id = "world"
            first_pose.header.stamp = self.get_clock().now().to_msg()

            # Position (x, y, z)
            first_pose.pose.position.x = 0.0
            first_pose.pose.position.y = 0.0
            first_pose.pose.position.z = 0.0

            # Identity orientation quaternion (x, y, z, w)
            first_pose.pose.orientation.x = 0.0
            first_pose.pose.orientation.y = 0.0
            first_pose.pose.orientation.z = 0.0
            first_pose.pose.orientation.w = 1.0

            # Add the pose to the path's poses array
            self.path.poses.append(first_pose)

            self.pose_publisher.publish(first_pose)
            self.path_publisher.publish(self.path)

            self.first_message = False

        R1, t1, points_a, points_b = compute_optical_flow(image_a, image_b, self.mtx)
        R2, t2, points_b_bc, points_c = compute_optical_flow(image_b, image_c, self.mtx)

        # camera a's pose in the world, needed to place pair 1's point cloud
        R_world_a, t_world_a = self.R_global, self.t_global

        # Pair 1 (image a, b) - pose and path
        # R1, t1 map camera a -> camera b, so camera b's pose in a is (R1.T, -R1.T @ t1)
        self.R_global = self.R_global @ R1.T
        self.t_global = self.t_global - self.R_global @ t1

        scipy_rotation = Rotation.from_matrix(self.R_global)
        global_quat_matrix = scipy_rotation.as_quat()
        global_translation_matrix = self.t_global.flatten()

        pose_ab = Pose()
        pose_ab.position = Point(x=global_translation_matrix[0], y=global_translation_matrix[1], z=global_translation_matrix[2])
        pose_ab.orientation = Quaternion(x=global_quat_matrix[0], y=global_quat_matrix[1], z=global_quat_matrix[2], w=global_quat_matrix[3])

        pose_stamped_ab = PoseStamped()
        pose_stamped_ab.header.stamp = msg.image_b.header.stamp
        pose_stamped_ab.header.frame_id = "world"
        pose_stamped_ab.pose = pose_ab

        self.path.poses.append(pose_stamped_ab)

        self.pose_publisher.publish(pose_stamped_ab)
        self.path_publisher.publish(self.path)

        # Pair 1 (image a, b) - tf broadcast
        transform_stamped_ab = TransformStamped()
        transform_stamped_ab.header.stamp = msg.image_b.header.stamp
        transform_stamped_ab.header.frame_id = "world"
        transform_stamped_ab.child_frame_id = "camera"

        transform_ab = Transform()
        transform_ab.translation = Vector3(x=global_translation_matrix[0], y=global_translation_matrix[1], z=global_translation_matrix[2])
        transform_ab.rotation = Quaternion(x=global_quat_matrix[0], y=global_quat_matrix[1], z=global_quat_matrix[2], w=global_quat_matrix[3])
        transform_stamped_ab.transform = transform_ab

        self.tf_broadcaster.sendTransform(transform_stamped_ab)

        point_field = [PointField(name = 'x', offset = 0, datatype = 7, count = 1),
                       PointField(name = 'y', offset = 4, datatype = 7, count = 1),
                       PointField(name = 'z', offset = 8, datatype = 7, count = 1)]

        # Pair 1 (image a, b) - point cloud (skipped until the camera is calibrated)
        if self.mtx is not None:
            point_cloud_matrix = compute_point_cloud(R1, t1, self.mtx, points_a, points_b)
            # camera a frame -> world frame
            point_cloud_matrix = (R_world_a @ point_cloud_matrix.T + t_world_a).T
            point_cloud_matrix = np.array(point_cloud_matrix, dtype=np.float32)

            point_cloud = PointCloud2()
            point_cloud.header.stamp = msg.header.stamp
            point_cloud.header.frame_id = "world"
            point_cloud.height = 1
            point_cloud.width = point_cloud_matrix.shape[0]
            point_cloud.fields = point_field
            point_cloud.is_bigendian = False
            point_cloud.point_step = 12
            point_cloud.row_step = 12 * point_cloud_matrix.shape[0]
            point_cloud.data = point_cloud_matrix.tobytes()
            point_cloud.is_dense = True

            self.point_cloud_publisher.publish(point_cloud)

        # camera b's pose in the world, needed to place pair 2's point cloud
        R_world_b, t_world_b = self.R_global, self.t_global

        # Pair 2 (image b, c) - pose and path
        self.R_global = self.R_global @ R2.T
        self.t_global = self.t_global - self.R_global @ t2

        scipy_rotation = Rotation.from_matrix(self.R_global)
        global_quat_matrix = scipy_rotation.as_quat()
        global_translation_matrix = self.t_global.flatten()

        pose_bc = Pose()
        pose_bc.position = Point(x=global_translation_matrix[0], y=global_translation_matrix[1], z=global_translation_matrix[2])
        pose_bc.orientation = Quaternion(x=global_quat_matrix[0], y=global_quat_matrix[1], z=global_quat_matrix[2], w=global_quat_matrix[3])

        pose_stamped_bc = PoseStamped()
        pose_stamped_bc.header.stamp = msg.image_c.header.stamp
        pose_stamped_bc.header.frame_id = "world"
        pose_stamped_bc.pose = pose_bc

        self.path.poses.append(pose_stamped_bc)

        self.pose_publisher.publish(pose_stamped_bc)
        self.path_publisher.publish(self.path)

        # Pair 2 (image b, c) - tf broadcast
        transform_stamped_bc = TransformStamped()
        transform_stamped_bc.header.stamp = msg.image_c.header.stamp
        transform_stamped_bc.header.frame_id = "world"
        transform_stamped_bc.child_frame_id = "camera"

        transform_bc = Transform()
        transform_bc.translation = Vector3(x=global_translation_matrix[0], y=global_translation_matrix[1], z=global_translation_matrix[2])
        transform_bc.rotation = Quaternion(x=global_quat_matrix[0], y=global_quat_matrix[1], z=global_quat_matrix[2], w=global_quat_matrix[3])
        transform_stamped_bc.transform = transform_bc

        self.tf_broadcaster.sendTransform(transform_stamped_bc)

        # Pair 2 (image b, c) - point cloud (skipped until the camera is calibrated)
        if self.mtx is not None:
            point_cloud_matrix_bc = compute_point_cloud(R2, t2, self.mtx, points_b_bc, points_c)
            # camera b frame -> world frame
            point_cloud_matrix_bc = (R_world_b @ point_cloud_matrix_bc.T + t_world_b).T
            point_cloud_matrix_bc = np.array(point_cloud_matrix_bc, dtype=np.float32)

            point_cloud_bc = PointCloud2()
            point_cloud_bc.header.stamp = msg.header.stamp
            point_cloud_bc.header.frame_id = "world"
            point_cloud_bc.height = 1
            point_cloud_bc.width = point_cloud_matrix_bc.shape[0]
            point_cloud_bc.fields = point_field
            point_cloud_bc.is_bigendian = False
            point_cloud_bc.point_step = 12
            point_cloud_bc.row_step = 12 * point_cloud_matrix_bc.shape[0]
            point_cloud_bc.data = point_cloud_matrix_bc.tobytes()
            point_cloud_bc.is_dense = True

            self.point_cloud_publisher.publish(point_cloud_bc)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()
