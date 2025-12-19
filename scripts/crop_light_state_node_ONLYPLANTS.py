import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np


class CropLightStateNode(Node):
    def __init__(self):
        super().__init__('crop_light_state_node')

        # --- Suscriptores ---
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.rgb_callback,
            10)
        
        self.mask_sub = self.create_subscription(
            Image,
            '/Temperature_and_CSWI/rescaled_yolo_masks',
            self.mask_callback,
            10)

        # --- Publicador ---
        self.publisher_ = self.create_publisher(String, 'crop_light_state', 10)

        # --- Utilidades ---
        self.bridge = CvBridge()
        self.latest_rgb = None

        # --- Parámetros de decisión ---
        self.bright_pixel_threshold = 130      # intensidad mínima considerada "brillante"
        self.bright_fraction_threshold = 0.35  # fracción mínima para considerar "sol"

        self.get_logger().info("CropLightStateNode (planta segmentada) iniciado.")

    # Guarda la última imagen RGB
    def rgb_callback(self, msg):
        self.latest_rgb = msg

    # Procesa la máscara cuando llega
    def mask_callback(self, msg):
        if self.latest_rgb is None:
            return  # aún no hay imagen RGB

        try:
            # Convertir mensajes ROS a imágenes OpenCV
            mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            rgb = self.bridge.imgmsg_to_cv2(self.latest_rgb, desired_encoding='bgr8')

            # Convertir a escala de grises
            gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)

            # Asegurar que la máscara y la imagen tienen el mismo tamaño
            if mask.shape != gray.shape:
                mask = cv2.resize(mask, (gray.shape[1], gray.shape[0]))

            # Seleccionar solo los píxeles de la planta
            plant_pixels = gray[mask > 128]

            if plant_pixels.size == 0:
                self.get_logger().warn("Máscara vacía o sin píxeles válidos.")
                return

            # Calcular fracción de píxeles brillantes dentro de la planta
            bright_fraction = np.mean(plant_pixels > self.bright_pixel_threshold)

            # Decidir estado
            state = "sun" if bright_fraction > self.bright_fraction_threshold else "shade"

            # Publicar resultado
            msg_out = String()
            msg_out.data = state
            self.publisher_.publish(msg_out)

            self.get_logger().info(
                f"Light state: {state} (bright_fraction={bright_fraction:.2f})"
            )

        except Exception as e:
            self.get_logger().error(f"Error procesando imagen: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CropLightStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

