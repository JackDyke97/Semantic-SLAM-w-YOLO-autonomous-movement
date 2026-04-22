import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
import math
import rclpy.time

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
MIN_DETECTIONS = 3

class SimFusionNode(Node):
    def __init__(self):
        super().__init__('sim_fusion_node')

        #use sim time to consistency
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.Parameter.Type.BOOL,
            True
            )])

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.latest_scan = None
        self.detection_counts = {}
        self.semantic_map = {}

        self.sub_detections = self.create_subscription(
            Detection2DArray, '/detections', self.detection_callback, 10
        )
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        self.pub_markers = self.create_publisher(MarkerArray, '/semantic_map/markers', 10)

        self.timer = self.create_timer(1.0, self.publish_markers)
        self.get_logger().info('Fusion node started')

    def scan_callback(self, msg):
        self.latest_scan = msg

    #use detections from scan to updatethe semantic map
    def detection_callback(self, msg):
        if self.latest_scan is None:
            return
        
        for det in msg.detections:
            if not det.results:
                continue

            class_name = det.results[0].hypothesis.class_id
            conf = det.results[0].hypothesis.score
            if conf < CONF_THRESHOLD:
                continue

            u = det.bbox.center.position.x
            angle_to_object = math.atan2(CX - u, FX)

            scan = self.latest_scan
            num_rays = len(scan.ranges)
            ray_angle = scan.angle_min
            best_idx = 0
            best_diff = float('inf')

            for i in range(num_rays):
                diff = abs(ray_angle - angle_to_object)
                if diff < best_diff:
                    best_diff = diff
                    best_idx = i
                ray_angle += scan.angle_increment
            
            distance = scan.ranges[best_idx]
            if not math.isfinite(distance) or distance < 0.1 or distance > 3.5:
                continue

            x_robot = distance * math.cos(angle_to_object)
            y_robot = distance * math.sin(angle_to_object)

            try:
                pose = PoseStamped()
                pose.header.frame_id = 'base_footprint'
                pose.header.stamp = rclpy.time.Time().to_msg()
                pose.pose.position.x = x_robot
                pose.pose.position.y = y_robot
                pose.pose.orientation.w = 1.0

                map_pose = self.tf_buffer.transform(pose, 'map', timeout=rclpy.duration.Duration(seconds=0.1))

                mx = map_pose.pose.position.x
                my = map_pose.pose.position.y

                if class_name not in self.detection_counts:
                    self.detection_counts[class_name] = 0

                self.detection_counts[class_name] += 1

                if self.detection_counts[class_name] >= MIN_DETECTIONS:
                    self.semantic_map[class_name] = (mx, my)
                    self.get_logger().info(f'map updated: {class_name} at ({mx: .2f}, {my: .2f})')
            
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

            
