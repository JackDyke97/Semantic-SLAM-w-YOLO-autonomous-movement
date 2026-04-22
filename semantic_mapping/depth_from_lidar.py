import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import math

FX = 530.4669406576809
FY = 530.4669406576809
CX = 320.5
CY = 240.5
WIDTH = 640
HEIGHT = 480
HFOV = 1.085595  # radians

class DepthFromLidar(Node):
    def __init__(self):
        super().__init__('depth_from_lidar')
        self.bridge = CvBridge()
        self.latest_scan = None

        #find lidar scans to use as depth
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.pub_depth = self.create_publisher(
            Image, '/camera/depth/image_raw', 10
        )

        #camera intrinsics for faking depth image
        self.pub_info = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 10)

        self.timer = self.create_timer(0.1, self.publish_depth)
        self.get_logger().info('Depth from LiDAR node started')


    def scan_callback(self, msg):
        self.latest_scan = msg

    #this functions converts the latest LiDAR scan into a depth image
    #then publishes it. Only necessary for sim due to depth bug in my installation
    def publish_depth(self):
        if self.latest_scan is None:
            return
        
        scan = self.latest_scan
        depth_image = np.zeros((HEIGHT, WIDTH), dtype=np.float32)
        half_hfov = HFOV / 2.0

        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
                continue

            angle = scan.angle_min + i * scan.angle_increment

            if abs(angle) > half_hfov:
                continue

            u = int(CX - FX * math.tan(angle))
            if u < 0 or u >= WIDTH:
                continue

            for v in range(HEIGHT):
                if depth_image[v, u] == 0 or r < depth_image[v, u]:
                    depth_image[v, u] = r
        
        for u in range(WIDTH):
            col = depth_image[:, u]
            nonzero = np.nonzero(col)[0]
            if len(nonzero) > 0:
                val = col[nonzero[0]]
                depth_image[:, u] = val

        msg_out = self.bridge.cv2_to_imgmsg(depth_image, encoding='32FC1')
        msg_out.header.stamp = self.latest_scan.header.stamp
        msg_out.header.frame_id = 'camera_rgb_optical_frame'
        self.pub_depth.publish(msg_out)

        info = CameraInfo()
        info.header.stamp = self.latest_scan.header.stamp
        info.width = WIDTH
        info.height = HEIGHT
        info.distortion_model = 'plumb_bob'
        info.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
        info.p = [FX, 0.0, CX, 0.0, 0.0, FY, CY, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.pub_info.publish(info)

def main(args=None):
    rclpy.init(args=args)
    node = DepthFromLidar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()