import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

TRACKED_CLASSES = ['bottle', 'can', 'dining table', 'bookcase', 'bowl', 'chair', 'bed', 'person', 'couch']

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.Parameter.Type.BOOL,
            True
            )])

        self.bridge = CvBridge()
        self.model = YOLO('yolo26n.pt')
        self.get_logger().info('YOLOv8 model loaded')

        # self.sub = self.create_subscription(
        #     Image, '/camera/image_raw', self.image_callback, 10)

        self.declare_parameter('image_topic', '/camera/image_raw')
        image_topic = self.get_parameter('image_topic').value
        self.sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)

        self.pub_detections = self.create_publisher(
            Detection2DArray, '/detections', 10)

        self.pub_image = self.create_publisher(
            Image, '/detections/image', 10)

    #runs YOLO on image and publishes what is detected to terminal
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model.predict(frame, conf=0.4, verbose=False)

        det_array = Detection2DArray()
        det_array.header = msg.header

        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]

                if class_name not in TRACKED_CLASSES:
                    continue

                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                w = x2 - x1
                h = y2 - y1

                det = Detection2D()
                det.header = msg.header
                det.bbox.center.position.x = cx
                det.bbox.center.position.y = cy
                det.bbox.size_x = w
                det.bbox.size_y = h

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = class_name
                hyp.hypothesis.score = conf
                det.results.append(hyp)
                det_array.detections.append(det)

                # Draw on frame
                cv2.rectangle(frame, (int(x1), int(y1)),
                              (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(frame, f'{class_name} {conf:.2f}',
                           (int(x1), int(y1) - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                self.get_logger().info(
                    f'Detected: {class_name} ({conf:.2f}) at ({cx:.0f}, {cy:.0f})')

        self.pub_detections.publish(det_array)
        annotated = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        annotated.header = msg.header
        self.pub_image.publish(annotated)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
