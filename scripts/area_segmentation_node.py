import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv2
from cv_bridge import CvBridge
import numpy as np

class AreaSegmentNode(Node):
    def __init__(self):
        super().__init__('area_segment_node')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/Temperature_and_CSWI/rescaled_yolo_masks',
            self.mask_callback,
            10
        )

        self.publisher = self.create_publisher(
            Float32,
            '/segmentation_area_info',
            10
        )

        self.get_logger().info("AreaSegmentNode started, listening to /rescaled_yolo_masks")

    def mask_callback(self, msg):
        # Convertimos la máscara a OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

        total_pixels = cv_image.shape[0] * cv_image.shape[1]
        segmented_pixels = np.count_nonzero(cv_image == 255)  # suponiendo binaria

        percentage = (segmented_pixels / total_pixels) * 100.0

        msg_out = Float32()
        msg_out.data = percentage

        self.publisher.publish(msg_out)

        self.get_logger().info(f"Segmented area: {percentage:.2f}%")

def main(args=None):
    rclpy.init(args=args)
    node = AreaSegmentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
