import math

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
import tf2_ros
import tf2_geometry_msgs  # noqa: F401
from tf2_geometry_msgs import do_transform_pose_stamped
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration


"""
This node is only needed when testing on the gazebo sim
if testing on real equipment, use the regular fusion_node
TO DO: create separate launch files
"""

# Simulation camera values from /camera/camera_info
FX = 530.4669406576809
FY = 530.4669406576809
CX = 320.5
CY = 240.5

CONF_THRESHOLD = 0.4
MIN_DETECTIONS = 1      
CLUSTER_DISTANCE = 0.5      
MAX_SCAN_RANGE = 3.5
MIN_SCAN_RANGE = 0.1


class SimFusionNode(Node):
    def __init__(self):
        super().__init__('sim_fusion_node')

        #use sim time to consistency
        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time',
                rclpy.Parameter.Type.BOOL,
                True
            )
        ])

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.latest_scan = None
        self.semantic_map = {}
        self.class_counts = {}
        self.instance_counts = {}

        self.sub_detections = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10
        )

        self.sub_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.pub_markers = self.create_publisher(
            MarkerArray,
            '/semantic_map/markers',
            10
        )

        self.timer = self.create_timer(1.0, self.publish_markers)
        self.get_logger().info('Sim fusion node started')

    def scan_callback(self, msg):
        self.latest_scan = msg

    def create_instance(self, class_name, mx, my):
        for key, (ex, ey) in self.semantic_map.items():
            if key.startswith(f'{class_name}_'):
                dist = math.sqrt((mx - ex) ** 2 + (my - ey) ** 2)
                if dist < CLUSTER_DISTANCE:
                    return key

        if class_name not in self.class_counts:
            self.class_counts[class_name] = 0

        self.class_counts[class_name] += 1
        new_key = f'{class_name}_{self.class_counts[class_name]}'
        self.instance_counts[new_key] = 0
        return new_key

    #use detections from scan to updatethe semantic map
    def detection_callback(self, msg):
        if self.latest_scan is None:
            self.get_logger().warn('No scan received yet', throttle_duration_sec=5.0)
            return

        scan = self.latest_scan
        num_rays = len(scan.ranges)

        for det in msg.detections:
            if not det.results:
                continue

            class_name = det.results[0].hypothesis.class_id
            conf = det.results[0].hypothesis.score

            if conf < CONF_THRESHOLD:
                continue

            u = float(det.bbox.center.position.x)

            angle_to_object = math.atan2((CX - u), FX)

            best_idx = 0
            best_diff = float('inf')

            for i in range(num_rays):
                ray_angle = scan.angle_min + i * scan.angle_increment
                diff = abs(ray_angle - angle_to_object)
                if diff < best_diff:
                    best_diff = diff
                    best_idx = i

            distance = scan.ranges[best_idx]

            if not math.isfinite(distance):
                continue
            if distance < MIN_SCAN_RANGE or distance > MAX_SCAN_RANGE:
                continue

            x_robot = distance * math.cos(angle_to_object)
            y_robot = distance * math.sin(angle_to_object)

            try:
                pose = PoseStamped()
                pose.header.frame_id = 'base_footprint'
                pose.header.stamp = msg.header.stamp
                pose.pose.position.x = x_robot
                pose.pose.position.y = y_robot
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0

                transform = self.tf_buffer.lookup_transform(
                    'map',
                    'base_footprint',
                    msg.header.stamp,
                    timeout=Duration(seconds=0.2)
                )

                map_pose = do_transform_pose_stamped(pose, transform)

                mx = map_pose.pose.position.x
                my = map_pose.pose.position.y

                instance_key = self.create_instance(class_name, mx, my)

                if instance_key not in self.instance_counts:
                    self.instance_counts[instance_key] = 0

                self.instance_counts[instance_key] += 1

                if self.instance_counts[instance_key] >= MIN_DETECTIONS:
                    self.semantic_map[instance_key] = (mx, my)
                    self.get_logger().info(
                        f'map updated: {instance_key} at ({mx:.2f}, {my:.2f})'
                    )

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
            sphere.scale.x = 0.15
            sphere.scale.y = 0.15
            sphere.scale.z = 0.15
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
            text.pose.position.z = 0.35
            text.pose.orientation.w = 1.0
            text.scale.z = 0.15
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = class_name
            marker_array.markers.append(text)

        self.pub_markers.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = SimFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()