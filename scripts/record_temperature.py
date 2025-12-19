import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import String
import csv
import threading
import os
import math
import re


class DataLoggerNode(Node):
    def __init__(self, filename):
        super().__init__('data_logger')

        self.ndvi = None
        self.gps = None
        self.heading_quat = None
        self.temp = None
        self.area = None

        self.create_subscription(String, '/NDVI', self.ndvi_callback, 10)
        self.create_subscription(String, '/Temperature_and_CSWI/text', self.temp_callback, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.create_subscription(Imu, '/ublox_rover/navheading', self.heading_callback, 10)
        self.create_subscription(String, '/segmentation_area_info', self.area_callback, 10)


        self.filename = filename
        self.lock = threading.Lock()

        with open(self.filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Timestamp', 'NDVI', 'Latitude', 'Longitude', 'Heading (deg)', 'Temperature'])

        self.get_logger().info(f"Logging to {self.filename}. Press ENTER to log data, or type 'exit' to quit.")

    def ndvi_callback(self, msg):
        with self.lock:
            try:
                self.ndvi = float(msg.data.split(',')[0].strip())  # Extract first float
                # self.get_logger().info(f"NDVI[0]: {self.ndvi}")
            except Exception as e:
                self.get_logger().warn(f"Failed to parse NDVI from: '{msg.data}' | Error: {e}")
                self.ndvi = None

    def temp_callback(self, msg):
        with self.lock:
            # Example: ' Objeto 1: Temperatura = 32.09 °C, CSWI = 2.02'
            try:
                match = re.search(r'Temperatura\s*=\s*([0-9.]+)', msg.data)
                if match:
                    self.temp = float(match.group(1))
                    # self.get_logger().info(f"Parsed Temperature: {self.temp:.2f} °C")
                else:
                    self.get_logger().warn(f"Could not parse temperature from: '{msg.data}'")
                    self.temp = None
            except Exception as e:
                self.get_logger().error(f"Error parsing temperature: {e}")
                self.temp = None

    def gps_callback(self, msg):
        with self.lock:
            self.gps = msg
            self.get_logger().info(f"GPS fix: lat = {msg.latitude}, lon = {msg.longitude}")
            
    def heading_callback(self, msg):
        with self.lock:
            self.heading_quat = msg.orientation
            try:
                yaw = self.quaternion_to_yaw(self.heading_quat)
                # self.get_logger().info(f"Heading (yaw): {yaw:.2f} degrees")
            except Exception as e:
                self.get_logger().warn(f"Failed to compute heading: {e}")


    def area_callback(self, msg):
        with self.lock:
            try:
                self.area = float(msg.data)
                self.get_logger().info(f"Segmented area: {self.area:.2f}%")
            except Exception as e:
                self.get_logger().warn(f"Failed to parse segmented area: {e}")
                self.area = None
            


    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw in degrees"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return math.degrees(yaw)

    def record_data(self):
        with self.lock:
            ros_time = self.get_clock().now().to_msg()
            timestamp = f"{ros_time.sec}.{str(ros_time.nanosec).zfill(9)}"
            ndvi = self.ndvi if self.ndvi is not None else 'NaN'
            lat = self.gps.latitude if self.gps else 'NaN'
            lon = self.gps.longitude if self.gps else 'NaN'
            heading = self.quaternion_to_yaw(self.heading_quat) if self.heading_quat else 'NaN'
            temp = self.temp if self.temp is not None else 'NaN'
            area = self.area if self.area is not None else 'NaN'

        with open(self.filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([timestamp, ndvi, lat, lon, heading, temp, area])
            self.get_logger().info(f"Logged data at {timestamp}")


def main(args=None):
    rclpy.init(args=args)

    filename = input("Enter CSV filename (e.g., data_log.csv): ").strip()
    if not filename.endswith('.csv'):
        filename += '.csv'

    node = DataLoggerNode(filename)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1) 
            user_input = input("Press ENTER to log data, or type 'exit' to quit: ").strip().lower()
            if user_input == 'exit':
                break
            node.record_data()
    except KeyboardInterrupt:
        print("\nShutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
