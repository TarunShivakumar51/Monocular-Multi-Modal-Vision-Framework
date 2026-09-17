import rclpy
from rclpy.node import Node
from mono_perception_msgs.msg import ImageTriple
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TransformStamped, Transform, Vector3
from sensor_msgs.msg import PointCloud2, PointField
from cv_bridge import CvBridge,CvBridgeError
from optical_flow import compute_optical_flow
from feature_matching import compute_keypoints
from triangulation import compute_point_cloud
import numpy as np
from scipy.spatial.transform import Rotation
from tf2_ros import TransformBroadcaster


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("image_subscriber")

        self.image_subscriber = self.create_subscription(ImageTriple, "/image_triples", self.create_path, 10)

        self.R_global = np.eye(3)
        self.t_global = np.zeros((3,1))
        self.mtx = None

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
            self.get_logger().info(e)

        path = Path()
        path.header = msg.header()

        R1, t1, good_old, good_new = compute_optical_flow(image_a, image_b)
        R2, t2, good_old, good_new = compute_optical_flow(image_b, image_c)

        # Pair 1 (image a, b) - pose and path
        self.t_global = np.add(self.t_global, np.matmul(self.R_global, t1))
        self.R_global = np.matmul(R1, self.R_global)

        scipy_rotation = Rotation.from_matrix(self.R_global)
        global_quat_matrix = scipy_rotation.as_quat()
        global_translation_matrix = self.t_global.flatten()

        pose_ab = Pose()
        pose_ab.position = Point(x=global_translation_matrix[0], y=global_translation_matrix[1], z=global_translation_matrix[2])
        pose_ab.orientation = Quaternion(x=global_quat_matrix[0], y=global_quat_matrix[1], z=global_quat_matrix[2], w=global_quat_matrix[3])

        pose_stamped_ab = PoseStamped()
        pose_stamped_ab.header = msg.header()
        pose_stamped_ab.pose = pose_ab

        path.poses.append(pose_stamped_ab)

        self.pose_publisher.publish(pose_stamped_ab)
        self.path_publisher.publish(path)

        # Pair 1 (image a, b) - tf broadcast
        transform_stamped_ab = TransformStamped()
        transform_stamped_ab.header.stamp = self.get_clock().now().to_msg()
        transform_stamped_ab.header.frame_id = "world"
        transform_stamped_ab.child_frame_id = "camera"

        transform_ab = Transform()
        transform_ab.translation = Vector3(x=global_translation_matrix[0], y=global_translation_matrix[1], z=global_translation_matrix[2])
        transform_ab.rotation = Quaternion(x=global_quat_matrix[0], y=global_quat_matrix[1], z=global_quat_matrix[2], w=global_quat_matrix[3])
        transform_stamped_ab.transform = transform_ab

        self.tf_broadcaster.sendTransform(transform_stamped_ab)

        # Pair 1 (image a, b) - point cloud
        frame_a_keypoints, frame_b_keypoints = compute_keypoints(image_a, image_b)
        point_cloud_matrix = compute_point_cloud(R1, t1, R2, t2, self.mtx, frame_a_keypoints, frame_b_keypoints)
        point_cloud_matrix = np.array(point_cloud_matrix, dtype=np.float32)

        point_field = [PointField(name = 'x', offset = 0, datatype = 7, count = 1),
                       PointField(name = 'y', offset = 4, datatype = 7, count = 1),
                       PointField(name = 'z', offset = 8, datatype = 7, count = 1)]

        point_cloud = PointCloud2()
        point_cloud.header = msg.header()
        point_cloud.height = 1
        point_cloud.width = point_cloud_matrix.shape[0]
        point_cloud.fields = point_field
        point_cloud.is_bigendian = False
        point_cloud.point_step = 12
        point_cloud.row_step = 12 * point_cloud_matrix.shape[0]
        point_cloud.data = point_cloud_matrix.tobytes()
        point_cloud.is_dense = True

        self.point_cloud_publisher.publish(point_cloud)

        # Pair 2 (image b, c) - pose and path
        self.t_global = np.add(self.t_global, np.matmul(self.R_global, t2))
        self.R_global = np.matmul(R2, self.R_global)

        scipy_rotation = Rotation.from_matrix(self.R_global)
        global_quat_matrix = scipy_rotation.as_quat()
        global_translation_matrix = self.t_global.flatten()

        pose_bc = Pose()
        pose_bc.position = Point(x=global_translation_matrix[0], y=global_translation_matrix[1], z=global_translation_matrix[2])
        pose_bc.orientation = Quaternion(x=global_quat_matrix[0], y=global_quat_matrix[1], z=global_quat_matrix[2], w=global_quat_matrix[3])

        pose_stamped_bc = PoseStamped()
        pose_stamped_bc.header = msg.header()
        pose_stamped_bc.pose = pose_bc

        path.poses.append(pose_stamped_bc)

        self.pose_publisher.publish(pose_stamped_bc)
        self.path_publisher.publish(path)

        # Pair 2 (image b, c) - tf broadcast
        transform_stamped_bc = TransformStamped()
        transform_stamped_bc.header.stamp = self.get_clock().now().to_msg()
        transform_stamped_bc.header.frame_id = "world"
        transform_stamped_bc.child_frame_id = "camera"

        transform_bc = Transform()
        transform_bc.translation = Vector3(x=global_translation_matrix[0], y=global_translation_matrix[1], z=global_translation_matrix[2])
        transform_bc.rotation = Quaternion(x=global_quat_matrix[0], y=global_quat_matrix[1], z=global_quat_matrix[2], w=global_quat_matrix[3])
        transform_stamped_bc.transform = transform_bc

        self.tf_broadcaster.sendTransform(transform_stamped_bc)

        # Pair 2 (image b, c) - point cloud
        frame_b_keypoints_bc, frame_c_keypoints = compute_keypoints(image_b, image_c)
        point_cloud_matrix_bc = compute_point_cloud(R1, t1, R2, t2, self.mtx, frame_b_keypoints_bc, frame_c_keypoints)
        point_cloud_matrix_bc = np.array(point_cloud_matrix_bc, dtype=np.float32)

        point_cloud_bc = PointCloud2()
        point_cloud_bc.header = msg.header()
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
