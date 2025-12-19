import re
import json
import time 
import paho.mqtt.client as mqtt
import math
import csv
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from datetime import datetime
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Temperature, RelativeHumidity
from geometry_msgs.msg import PointStamped
from tf2_msgs.msg import TFMessage

# MQTT broker configuration
MQTT_DEFAULT_HOST = "localhost"  
MQTT_DEFAULT_PORT = 1883         
MQTT_DEFAULT_TIMEOUT = 120       

# ROS2 topics to subscribe to
ROS2_TOPIC_GPS = "gps/fix"
ROS2_TOPIC_TEMPERATURE = "/Temperature_and_CSWI/text"
ROS2_TOPIC_NDVI = "/NDVI"
ROS2_TOPIC_HEADING = '/ublox_rover/navheading'
ROS2_TOPIC_AREA = '/segmentation_area_info'
ROS2_TOPIC_LOCATION = '/navigation/information'
ROS2_TOPIC_BIOMASS = '/plant_biomass'          
ROS2_TOPIC_CROP_LIGHT_STATE = '/crop_light_state'
ROS2_TOPIC_CROP_TYPE = '/crop_type'
ROS2_TOPIC_AMBIENT_TEMPERATURE = 'pce_p18/temperature'
ROS2_TOPIC_RELATIVE_HUMIDITY = 'pce_p18/rel_humidity' 
ROS2_TOPIC_ABSOLUTE_HUMIDITY = 'pce_p18/abs_humidity'
ROS2_TOPIC_DEW_POINT = 'pce_p18/dew_point'
ROS2_TOPIC_UTM_BASELINK = '/tf_utm_baselink'

# MQTT global topic
MQTT_GLOBAL_TOPIC = "mqtt/global"

# Regex
GET_FLOAT_NUMBER = r'\d+\.\d+'
GET_TEMPERATURE_DATA = rf'([^\":]+):|({GET_FLOAT_NUMBER})'

class ros2_mqtt_publisher_t(Node):
    def __init__(self, host=MQTT_DEFAULT_HOST, port=MQTT_DEFAULT_PORT):
        super().__init__('ros2_mqtt_publisher')

        # MQTT setup
        self.is_connected = False
        self.pending_messages = {}
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_server_connection
        self.mqtt_client.on_disconnect = self.on_disconnect
        self.mqtt_client.on_publish = self.on_publish

        try:
            self.mqtt_client.connect(host, port, MQTT_DEFAULT_TIMEOUT)
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT broker: {e}")

        # Start MQTT loop
        self.mqtt_client.loop_start()

        # Subscribe to mqtt/global for logging
        self.mqtt_client.message_callback_add(MQTT_GLOBAL_TOPIC, self.mqtt_global_callback)
        self.mqtt_client.subscribe(MQTT_GLOBAL_TOPIC, qos=1)

        # CSV setup
        home = os.path.expanduser("~/sensors_ws/data_collection")
        os.makedirs(home, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file = os.path.join(home, f"Data_collection_{timestamp}.csv")
        self.csv_fields = [
            "ts",
            "gps_lat","gps_lon","gps_alt", "gps_service", "gps_status",
            "temperature_canopy","temperature_cwsi",
            "ndvi", "ndvi3d", "ndvi_ir", "ndvi_visible","heading_deg","area","location",
            "biomass","crop_light_state","crop_type",
            "ambient_temperature","relative_humidity","absolute_humidity", "dew_point",
            "tf_utm_baselink_X","tf_utm_baselink_Y"
        ]
        self.latest_row = {k: "" for k in self.csv_fields}

        with open(self.csv_file, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self.csv_fields)
            writer.writeheader()

        # ROS2 subscriptions (optional, only if you still want them for MQTT publishing)
        self.subscribe_to_topics()

    # ------------------- MQTT callbacks -------------------
    def on_server_connection(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Connected to MQTT broker.")
            self.is_connected = True
            self.resend_local_backups()
        else:
            self.get_logger().error(f"Connection failed with result code={rc}")
            self.is_connected = False

    def on_disconnect(self, client, userdata, rc):
        self.get_logger().warn("Disconnected from MQTT broker.")
        self.is_connected = False

    def on_publish(self, client, userdata, mid):
        if mid in self.pending_messages:
            del self.pending_messages[mid]

    # ------------------- MQTT global logging -------------------
    def mqtt_global_callback(self, client, userdata, msg):
        payload = msg.payload.decode()  # bytes → str
        self.get_logger().info(f"Received from mqtt/global: {payload}")


    # ------------------- ROS2 subscriptions -------------------
    def subscribe_to_topics(self):
        qos = QoSProfile(depth=10)
        self.create_subscription(NavSatFix, ROS2_TOPIC_GPS, self.gps_callback, qos)
        self.create_subscription(String, ROS2_TOPIC_TEMPERATURE, self.temperature_callback, qos)
        self.create_subscription(String, ROS2_TOPIC_NDVI, self.ndvi_callback, qos)  
        self.create_subscription(Imu, ROS2_TOPIC_HEADING, self.heading_callback, 10)
        self.create_subscription(Float32, ROS2_TOPIC_AREA, self.area_callback, 10)
        self.create_subscription(String, ROS2_TOPIC_LOCATION, self.location_callback, 10)
        self.create_subscription(Float32, ROS2_TOPIC_BIOMASS, self.biomass_callback, 10)
        self.create_subscription(String, ROS2_TOPIC_CROP_LIGHT_STATE, self.crop_light_state_callback, 10)
        self.create_subscription(String, ROS2_TOPIC_CROP_TYPE, self.crop_type_callback, 10)
        self.create_subscription(Temperature, ROS2_TOPIC_AMBIENT_TEMPERATURE, self.ambient_temperature_callback, 10)
        self.create_subscription(RelativeHumidity, ROS2_TOPIC_RELATIVE_HUMIDITY, self.relative_humidity_callback, 10)
        self.create_subscription(Temperature, ROS2_TOPIC_ABSOLUTE_HUMIDITY, self.absolute_humidity_callback, 10)
        self.create_subscription(Temperature, ROS2_TOPIC_DEW_POINT, self.dew_point_callback, 10)
        self.create_subscription(PointStamped, ROS2_TOPIC_UTM_BASELINK, self.utm_baselink_callback, 10)

    # ------------------- ROS2 → MQTT publishing -------------------
    def publish(self, topic: str, data: dict, qos: int = 1):
        data["ts"] = int(time.time())
        payload = json.dumps(data)

        # --- Save to CSV locally ---
        self.latest_row["ts"] = data["ts"]

        if data["msg_type"] == "gps":
            self.latest_row["gps_lat"] = data.get("latitude", "")
            self.latest_row["gps_lon"] = data.get("longitude", "")
            self.latest_row["gps_alt"] = data.get("altitude", "")
            self.latest_row["gps_status"] = data.get("status", "")
            self.latest_row["gps_service"] = data.get("service", "")
        elif data["msg_type"] == "temperature":
            self.latest_row["temperature_canopy"] = data.get("canopy_temperature", "")
            self.latest_row["temperature_cwsi"] = data.get("cwsi", "")
        elif data["msg_type"] == "ndvi":
                    self.latest_row["ndvi"] = data.get("ndvi", "")
                    self.latest_row["ndvi3d"] = data.get("ndvi3d", "")     
                    self.latest_row["ndvi_ir"] = data.get("ndvi_ir", "")     
                    self.latest_row["ndvi_visible"] = data.get("ndvi_visible", "") 
        elif data["msg_type"] == "heading":
            self.latest_row["heading_deg"] = data.get("heading_deg", "")
        elif data["msg_type"] == "area":
            self.latest_row["area"] = data.get("area", "")
        elif data["msg_type"] == "location":
            self.latest_row["location"] = data.get("location", "")
        elif data["msg_type"] == "biomass":
            self.latest_row["biomass"] = data.get("biomass", "")
        elif data["msg_type"] == "light_state":
            self.latest_row["crop_light_state"] = data.get("crop_light_state", "")
        elif data["msg_type"] == "crop_type":
            self.latest_row["crop_type"] = data.get("crop_type", "")
        elif data["msg_type"] == "ambient_temperature":
            self.latest_row["ambient_temperature"] = data.get("ambient_temperature", "")
        elif data["msg_type"] == "relative_humidity":
            self.latest_row["relative_humidity"] = data.get("relative_humidity", "")
        elif data["msg_type"] == "absolute_humidity":
            self.latest_row["absolute_humidity"] = data.get("absolute_humidity", "")
        elif data["msg_type"] == "dew_point":
            self.latest_row["dew_point"] = data.get("dew_point", "")
        elif data["msg_type"] == "tf_position":
            self.latest_row["tf_utm_baselink_X"] = data.get("x", "")
            self.latest_row["tf_utm_baselink_Y"] = data.get("y", "")


        with open(self.csv_file, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self.csv_fields)
            writer.writerow(self.latest_row)


        # --- Publish to MQTT ---
        if self.is_connected:
            result = self.mqtt_client.publish(topic, payload, qos=qos)
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                self.pending_messages[result.mid] = (topic, payload)
            else:
                self.save_message_locally(topic, payload)
        else:
            self.save_message_locally(topic, payload)


    def save_message_locally(self, topic: str, payload: str):
        timestamp = int(time.time())
        entry = {"topic": topic, "payload": payload, "ts": timestamp}
        with open("pending_messages.jsonl", "a") as f:
            f.write(json.dumps(entry) + "\n")

    def check_pending(self, timeout: float = 5.0):
        start_time = time.time()
        while time.time() - start_time < timeout and self.pending_messages:
            time.sleep(0.1)
        for mid, (topic, payload) in list(self.pending_messages.items()):
            self.save_message_locally(topic, payload)
            del self.pending_messages[mid]

    def resend_local_backups(self):
        try:
            with open("pending_messages.jsonl", "r") as f:
                entries = [json.loads(line) for line in f if line.strip()]
        except FileNotFoundError:
            return
        entries.sort(key=lambda e: e["ts"])
        failed = []
        for entry in entries:
            result = self.mqtt_client.publish(entry["topic"], entry["payload"], qos=1)
            if result.rc != mqtt.MQTT_ERR_SUCCESS:
                failed.append(entry)
        with open("pending_messages.jsonl", "w") as f:
            for e in failed:
                f.write(json.dumps(e) + "\n")

    def gps_callback(self, msg: NavSatFix) -> None:
        """
        Process GPS messages, build a dict including sensor timestamp and publish timestamp, then send via MQTT.
        """
        sanitized = {
            "msg_type": "gps",
            "sensor_ts": msg.header.stamp.sec,  # original sensor timestamp
            "latitude": msg.latitude,
            "longitude": msg.longitude,
            "altitude": msg.altitude,
            "status": msg.status.status,
            "service": msg.status.service
        }
        self.publish(MQTT_GLOBAL_TOPIC, sanitized)
        self.check_pending()


    def temperature_callback(self, msg: String) -> None:
        """
        Procesa datos de temperatura y CWSI de múltiples objetos detectados.
        Cada mensaje tiene formato: 'Objeto N: Temperatura = XX.XX °C, CSWI = X.XX'
        El nodo mantiene un buffer de los últimos valores por objeto.
        """
        if not hasattr(self, "temperature_data"):
            self.temperature_data = {}  # inicializa si no existe

        try:
            # Extraer datos con regex robusto
            match = re.search(r'Objeto\s*(\d+).*?Temperatura\s*=\s*(-?[\d.]+).*?CSWI\s*=\s*(-?[\d.]+)', msg.data)
            if not match:
                self.get_logger().warn(f"Temperature format not recognized : {msg.data}")
                return

            obj_id, temp, cwsi = match.groups()
            obj_id = f"Objeto_{obj_id.strip()}"
            temp = float(temp)
            cwsi = float(cwsi)

            # Actualizar o agregar objeto
            self.temperature_data[obj_id] = {
                "canopy_temperature": temp,
                "cwsi": cwsi,
                "timestamp": int(time.time())
            }

            # Construir lista de todos los objetos activos
            all_objects = [
                {"id": k, "canopy_temperature": v["canopy_temperature"], "cwsi": v["cwsi"]}
                for k, v in self.temperature_data.items()
            ]

            # Publicar en MQTT
            sanitized = {
                "msg_type": "temperature",
                "entity_count": len(all_objects),
                "plants": all_objects
            }
            self.publish(MQTT_GLOBAL_TOPIC, sanitized)

            # Actualizar CSV con promedios
            avg_temp = sum(o["canopy_temperature"] for o in all_objects) / len(all_objects)
            avg_cwsi = sum(o["cwsi"] for o in all_objects) / len(all_objects)
            self.latest_row["temperature_canopy"] = round(avg_temp, 3)
            self.latest_row["temperature_cwsi"] = round(avg_cwsi, 3)

            self.check_pending()

        except Exception as e:
            self.get_logger().warn(f"Error procesando mensaje de temperatura: {e}")



    def ndvi_callback(self, msg: String):
            try:
                values = [float(v.strip()) for v in msg.data.split(',')]
                if len(values) >= 4:
                    sanitized = {
                        "msg_type": "ndvi",
                        "ndvi": values[0],
                        "ndvi_3d": values[1],
                        "ndvi_ir": values[2],
                        "ndvi_visible": values[3]
                    }
                    self.publish(MQTT_GLOBAL_TOPIC, sanitized)
                    self.check_pending()
                else:
                    self.get_logger().warn(f"NDVI data incomplete: {msg.data}")
            except ValueError as e:
                self.get_logger().error(f"Failed to parse NDVI values: {e}")
    

    def area_callback(self, msg: Float32):
        sanitized = {
            "msg_type": "area",
            "area": msg.data  
        }
        self.publish(MQTT_GLOBAL_TOPIC, sanitized)
        self.check_pending()

    def heading_callback(self, msg: Imu) -> None:
        """
        Parse heading (yaw) from IMU orientation quaternion, sanitize, and publish.
        """
        self.heading_quat = msg.orientation
        try:
            yaw_deg = self.quaternion_to_yaw(self.heading_quat)
            sanitized = {
                "msg_type": "heading",
                "heading_deg": yaw_deg
            }
            self.publish(MQTT_GLOBAL_TOPIC, sanitized)
            self.check_pending()
        except Exception as e:
            self.get_logger().warn(f"Failed to compute heading: {e}")

    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw in degrees"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return math.degrees(yaw)
                 
        
    def location_callback(self, msg: String) -> None:
        """
        Receive location data (String), wrap into MQTT JSON, and publish.
        """
        try:
            sanitized = {
                "msg_type": "location",
                "location": msg.data
            }
            self.publish(MQTT_GLOBAL_TOPIC, sanitized)
            self.check_pending()
        except Exception as e:
            self.get_logger().warn(f"Failed to process location data: {e}")


    def biomass_callback(self, msg: Float32) -> None:
        """
        Receive biomass estimation (Float32 in m³), wrap into MQTT JSON, and publish.
        """
        try:
            sanitized = {
                "msg_type": "biomass",
                "biomass": msg.data  
            }
            self.publish(MQTT_GLOBAL_TOPIC, sanitized)
            self.check_pending()
        except Exception as e:
            self.get_logger().warn(f"Failed to process biomass data: {e}")


    def crop_light_state_callback(self, msg: String) -> None:
        """
        Receive crop light state (String: 'sun' or 'shade'), wrap into MQTT JSON, and publish.
        """
        try:
            sanitized = {
                "msg_type": "light_state",
                "light_state": msg.data,
                "crop_light_state": msg.data  
            }
            self.publish(MQTT_GLOBAL_TOPIC, sanitized)
            self.check_pending()
        except Exception as e:
            self.get_logger().warn(f"Failed to process crop light state data: {e}")


    def crop_type_callback(self, msg: String) -> None:
        """
        Receive crop type (String: 'lettuce' or 'other_crop'),
        wrap into MQTT JSON, and publish.
        """
        try:
            sanitized = {
                "msg_type": "crop_type",
                "crop_type": msg.data
            }
            self.publish(MQTT_GLOBAL_TOPIC, sanitized)
            self.check_pending()
        except Exception as e:
            self.get_logger().warn(f"Failed to process crop type data: {e}")


    def ambient_temperature_callback(self, msg: Temperature) -> None:
        sanitized = {
            "msg_type": "ambient_temperature",
            "ambient_temperature": msg.temperature
        }
        self.publish(MQTT_GLOBAL_TOPIC, sanitized)
        self.check_pending()



    def relative_humidity_callback(self, msg: RelativeHumidity) -> None:
        sanitized = {
            "msg_type": "relative_humidity",
            "relative_humidity": msg.relative_humidity
        }
        self.publish(MQTT_GLOBAL_TOPIC, sanitized)
        self.check_pending()


    def absolute_humidity_callback(self, msg: Temperature) -> None:
        sanitized = {
            "msg_type": "absolute_humidity",
            "absolute_humidity": msg.temperature
        }
        self.publish(MQTT_GLOBAL_TOPIC, sanitized)
        self.check_pending()



    def dew_point_callback(self, msg: Temperature) -> None:
        sanitized = {
            "msg_type": "dew_point",
            "dew_point": msg.temperature
        }
        self.publish(MQTT_GLOBAL_TOPIC, sanitized)
        self.check_pending()


    def utm_baselink_callback(self, msg: PointStamped):
        sanitized = {
            "msg_type": "tf_position",
            "x": msg.point.x,
            "y": msg.point.y,
            "z": msg.point.z
        }
        self.publish(MQTT_GLOBAL_TOPIC, sanitized)
        self.check_pending()



# ------------------- Main -------------------
def main(args=None):
    rclpy.init(args=args)
    node = ros2_mqtt_publisher_t("localhost", 1883)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.check_pending(timeout=2)
        node.mqtt_client.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
