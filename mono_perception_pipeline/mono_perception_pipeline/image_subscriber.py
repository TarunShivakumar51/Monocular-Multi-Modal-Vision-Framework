import rclpy
from rclpy.node import Node
from mono_perception_msgs.msg import ImageTriple
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import PointCloud2, PointField
from cv_bridge import CvBridge,CvBridgeError
from optical_flow import compute_optical_flow
from feature_matching import compute_keypoints
from triangulation import compute_point_cloud
import numpy as np
from scipy.spatial.transform import Rotation


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("image_subscriber")

        self.image_subscriber = self.create_subscription(ImageTriple, "/image_triples", self.create_path, 10)

        self.R_global = np.eye(3)
        self.t_global = np.zeros((3,1))
        self.mtx = None

        self.bridge = CvBridge()

        self.path_publisher = self.create_publisher(Path, '/camera/path', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/camera/pose', 10)

    def create_path(self, msg: ImageTriple):
        
        try:
            image_a = self.bridge.imgmsg_to_cv2(msg.image_a, "bgr8")
            image_b = self.bridge.imgmsg_to_cv2(msg.image_b, "bgr8")
            image_c = self.bridge.imgmsg_to_cv2(msg.image_c, "bgr8")
        except CvBridgeError as e:
            self.get_logger().info(e)

        path = Path()
        path.header = msg.header()

        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header()

        pose = Pose()

        point = Point()
        quat = Quaternion()

        R1, t1, good_old, good_new = compute_optical_flow(image_a, image_b)
        R2, t2, good_old, good_new = compute_optical_flow(image_b, image_c)

        # Pair 1
        self.t_global = np.add(self.t_global, np.matmul(self.R_global, t1))
        self.R_global = np.matmul(R1, self.R_global)

        scipy_rotation = Rotation.from_matrix(self.R_global)
        global_quat_matrix = scipy_rotation.as_quat()
        global_translation_matrix = self.t_global.flatten()
        
        point.x = global_translation_matrix[0]
        point.y = global_translation_matrix[1]
        point.z = global_translation_matrix[2]
        
        quat.x = global_quat_matrix[0]
        quat.y = global_quat_matrix[1]
        quat.z = global_quat_matrix[2]
        quat.w = global_quat_matrix[3]

        pose.position = point
        pose.orientation = quat

        pose_stamped = pose

        path.poses.append(pose_stamped)

        self.pose_publisher.publish(pose)
        self.path_publisher.publish(path)

        #Pair 2
        self.t_global = np.add(self.t_global, np.matmul(self.R_global, t2))
        self.R_global = np.matmul(R2, self.R_global)
                
        point.x = self.t_global[0]
        point.y = self.t_global[1]
        point.z = self.t_global[2]
                
        quat.x = self.R_global[0]
        quat.y = self.R_global[1]
        quat.z = self.R_global[2]
        quat.w = self.R_global[3]
        
        pose.position = point
        pose.orientation = quat
        
        pose_stamped = pose

        path.poses.append(pose_stamped)

        self.pose_publisher.publish(pose)
        self.path_publisher.publish(path)

        frame_a_keypoints, frame_b_keypoints = compute_keypoints(image_a, image_b)
        point_cloud_matrix = compute_point_cloud(R1, t1, R2, t2, self.mtx, frame_a_keypoints, frame_b_keypoints)
        point_cloud_matrix = np.array(point_cloud_matrix, dtype=np.int32)

        point_field = [PointField(name = 'x', offset = 0, datatype = 7, count = 4),
                       PointField(name = 'y', offset = 3, datatype = 7, count = 4),
                       PointField(name = 'z', offset = 7, datatype = 7, count = 4)]

        point_cloud = PointCloud2()
        point_cloud.header = msg.header()
        point_cloud.height = 1
        point_cloud.width = point_cloud_matrix.width
        point_cloud.fields = point_field
        point_cloud.is_bigendian = False
        point_cloud.point_step = 12
        point_cloud.row_step = 12 * point_cloud_matrix.width
        point_cloud.data = point_cloud_matrix
        point_cloud.is_dense = True

        







            

        

                



        

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()
