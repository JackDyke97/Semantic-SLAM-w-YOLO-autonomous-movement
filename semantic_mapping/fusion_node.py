import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import MarkerArray, Marker
import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped
from geometry_msgs.msg import PoseStamped
import math
import rclpy.time
import numpy as np
from cv_bridge import CvBridge

"""
Use this node for testing on real equipment 
"""

FX = 910.9061279296875
FY = 910.6283569335938
CX = 644.8796997070312
CY = 352.79034423828125
CONF_THRESHOLD = 0.4
MIN_DETECTIONS = 3

class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.latest_depth = None
        self.detection_counts = {}
        self.semantic_map = {}

        self.sub_detections = self.create_subscription(
            Detection2DArray, '/detections', self.detection_callback, 10)
        self.sub_depth = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback, 10)

        self.pub_markers = self.create_publisher(
            MarkerArray, '/semantic_map/markers', 10)
        self.timer = self.create_timer(1.0, self.publish_markers)
        self.get_logger().info('Fusion node started')

    def depth_callback(self, msg):
        self.latest_depth = msg

    #use detections from scan to updatethe semantic map
    def detection_callback(self, msg):
        if self.latest_depth is None:
            self.get_logger().warn('No depth image yet', throttle_duration_sec=5.0)
            return

        depth_image = self.bridge.imgmsg_to_cv2(
            self.latest_depth, desired_encoding='passthrough')

        for det in msg.detections:
            if not det.results:
                continue

            class_name = det.results[0].hypothesis.class_id
            conf = det.results[0].hypothesis.score
            if conf < CONF_THRESHOLD:
                continue

            u = int(det.bbox.center.position.x)
            v = int(det.bbox.center.position.y)

            u = max(0, min(u, depth_image.shape[1] - 1))
            v = max(0, min(v, depth_image.shape[0] - 1))

            window = depth_image[
                max(0, v-5):min(depth_image.shape[0], v+5),
                max(0, u-5):min(depth_image.shape[1], u+5)
            ]
            valid = window[window > 0]
            if len(valid) == 0:
                continue

            distance = float(np.median(valid)) / 1000.0

            if distance < 0.1 or distance > 5.0:
                continue

            x_cam = (u - CX) * distance / FX
            y_cam = (v - CY) * distance / FY
            z_cam = distance

            self.get_logger().info(
                f'{class_name}: depth={distance:.2f}m, '
                f'cam=({x_cam:.2f},{y_cam:.2f},{z_cam:.2f})')

            try:
                pose = PoseStamped()
                pose.header.frame_id = 'camera_link'
                pose.header.stamp = rclpy.time.Time().to_msg()
                pose.pose.position.x = z_cam
                pose.pose.position.y = -x_cam
                pose.pose.position.z = -y_cam
                pose.pose.orientation.w = 1.0

                transform = self.tf_buffer.lookup_transform(
                    'map', 'camera_link',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5))
                map_pose = do_transform_pose_stamped(pose, transform)

                mx = map_pose.pose.position.x
                my = map_pose.pose.position.y

                if class_name not in self.detection_counts:
                    self.detection_counts[class_name] = 0
                self.detection_counts[class_name] += 1

                if self.detection_counts[class_name] >= MIN_DETECTIONS:
                    self.semantic_map[class_name] = (mx, my)
                    self.get_logger().info(
                        f'Map updated: {class_name} at ({mx:.2f}, {my:.2f})')

            except Exception as e:
                self.get_logger().warn(f'TF error: {str(e)}')

    def publish_markers(self):
        marker_array = MarkerArray()
        for i, (class_name, (x, y)) in enumerate(self.semantic_map.items()):
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = 'objects'
            sphere.id = i * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.position.z = 0.1
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.2
            sphere.scale.y = 0.2
            sphere.scale.z = 0.2
            sphere.color.r = 1.0
            sphere.color.g = 0.5
            sphere.color.b = 0.0
            sphere.color.a = 1.0
            marker_array.markers.append(sphere)

            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = 'labels'
            text.id = i * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = 0.4
            text.pose.orientation.w = 1.0
            text.scale.z = 0.2
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = class_name
            marker_array.markers.append(text)

        self.pub_markers.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()