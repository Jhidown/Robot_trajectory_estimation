import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Quaternion

# ===== Camera Parameters =====
FOCAL_MATRIX = np.array([[825.7095, 0, 416.1148],
                         [0, 837.0046, 490.3529],
                         [0, 0, 1]])
DEFAULT_SCALE = 0.3


class VisualOdometry(Node):
    def __init__(self):
        super().__init__('vo_node')

        # Sub / Pub
        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Point, 'vo/position', 10)
        self.path_pub = self.create_publisher(Path, 'vo/path', 10)
        self.odom_pub = self.create_publisher(Odometry, 'vo/odom', 10)  # NEW

        # Path storage
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"   

        # CV Bridge
        self.bridge = CvBridge()

        # VO internals
        self.focal_matrix = FOCAL_MATRIX
        self.scale = DEFAULT_SCALE
        self.pose = np.eye(4)
        self.trajectory = []
        self.prev_img = None
        self.prev_kp = None
        self.prev_des = None
        self.origin_point = None

        self.sift = cv2.ORB_create()
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.last_position = np.array([0.0, 0.0])

        self.get_logger().info("VO node started.")

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        pos = self.update_pose(gray)
        if pos is not None:
            now = self.get_clock().now().to_msg()

            # --- Publier position simple (Point)
            point_msg = Point()
            point_msg.x = float(pos[0])
            point_msg.y = float(pos[1])
            point_msg.z = 0.0
            self.publisher.publish(point_msg)

            # --- Publier Odometry (EKF input)
            odom_msg = Odometry()
            odom_msg.header.stamp = now
            odom_msg.header.frame_id = "map"
            odom_msg.child_frame_id = "base_link"
            odom_msg.pose.pose.position.x = float(pos[0])
            odom_msg.pose.pose.position.y = float(pos[1])
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation = Quaternion(w=1.0)  # VO 2D, orientation non estimée
            # Covariance grossière (à ajuster selon fiabilité VO)
            odom_msg.pose.covariance = [0.1, 0, 0, 0, 0, 0,
                                        0, 0.1, 0, 0, 0, 0,
                                        0, 0, 99999, 0, 0, 0,
                                        0, 0, 0, 99999, 0, 0,
                                        0, 0, 0, 0, 99999, 0,
                                        0, 0, 0, 0, 0, 0.5]
            self.odom_pub.publish(odom_msg)

            # --- Publier Path (pour debug/visu RViz)
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = now
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose = odom_msg.pose.pose

            self.path_msg.header.stamp = now
            self.path_msg.poses.append(pose_stamped)
            self.path_pub.publish(self.path_msg)

    def update_pose(self, gray_frame):
        kp, des = self.sift.detectAndCompute(gray_frame, None)

        if self.prev_img is None or des is None or self.prev_des is None:
            self.prev_img, self.prev_kp, self.prev_des = gray_frame, kp, des
            return None

        matches = self.bf.knnMatch(self.prev_des, des, k=2)
        pts1, pts2 = [], []
        for m, n in matches:
            if m.distance < 0.75 * n.distance:
                pts1.append(self.prev_kp[m.queryIdx].pt)
                pts2.append(kp[m.trainIdx].pt)

        if len(pts1) < 8:
            self.prev_img, self.prev_kp, self.prev_des = gray_frame, kp, des
            return None

        pts1, pts2 = np.array(pts1, dtype=np.float32), np.array(pts2, dtype=np.float32)

        E, mask = cv2.findEssentialMat(pts2, pts1, self.focal_matrix, cv2.RANSAC, 0.999, 1.0)
        if E is None:
            return None

        pts1_inliers = pts1[mask.ravel() == 1]
        pts2_inliers = pts2[mask.ravel() == 1]
        retval, R, t, _ = cv2.recoverPose(E, pts1_inliers, pts2_inliers, self.focal_matrix)

        if retval < 30 or np.linalg.norm(t) < 1e-3:
            return None

        scale = self.scale / (np.linalg.norm(t) + 1e-8)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = (scale * t).reshape(3)
        self.pose = self.pose @ T

        trans = self.pose[:3, 3]
        point = np.array([-trans[2], -trans[0]])

        if self.origin_point is None:
            self.origin_point = point

        shifted_point = point - self.origin_point
        self.trajectory.append(shifted_point)

        self.prev_img, self.prev_kp, self.prev_des = gray_frame, kp, des
        return shifted_point


def main(args=None):
    rclpy.init(args=args)
    node = VisualOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

