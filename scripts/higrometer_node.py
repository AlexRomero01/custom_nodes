# ROS2 node to read PCE-P18 sensor by Modbus RTU

# All the regsiters at once (registers need to be adjacent)
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from sensor_msgs.msg import Temperature, RelativeHumidity

import struct
from pymodbus.client import ModbusSerialClient

class PceP18Node(Node):
    def __init__(self):
        super().__init__('pce_p18_node')

        # Paramters
        self.port = '/dev/tempsensor'      # needs to be changed!! (depends on computer assignment)
        self.device_id = 1
        self.timer_period = 2.0  # seconds

        # Modbus registers to read - adresses on datasheet table 7
        self.base_address = 7501
        self.num_registers = 4      # T, RH, Td, AH

        # Starting Modbus client
        self.client = ModbusSerialClient(
            port=self.port,
            timeout=1,
            baudrate=9600,
            bytesize=8,
            parity='N',
            stopbits=1
        )
        self.client.connect()
        self.get_logger().info("Modbus client created. Sensor already connected.")

        # Publishers
        self.multi_pub = self.create_publisher(Float32MultiArray, 'pce_p18/data', 10)
        self.temp_pub = self.create_publisher(Temperature, 'pce_p18/temperature', 10)
        self.rh_pub = self.create_publisher(RelativeHumidity, 'pce_p18/rel_humidity', 10)
        self.dew_pub = self.create_publisher(Temperature, 'pce_p18/dew_point', 10)
        self.ah_pub = self.create_publisher(Temperature, 'pce_p18/abs_humidity', 10)
        # self.dew_pub = self.create_publisher(Float32, 'pce_p18/dew_point', 10)
        # self.ah_pub = self.create_publisher(Float32, 'pce_p18/abs_humidity', 10)

        # Timer
        self.timer = self.create_timer(self.timer_period, self.read_sensor)

    def read_sensor(self):
        try:
            rr = self.client.read_holding_registers(address=self.base_address, count=self.num_registers, device_id=self.device_id)

            if rr.isError() or not hasattr(rr, 'registers'):
                self.get_logger().warn(f'Error reading Modbus registers.')
                return

            values = []
            for i in range(0, len(rr.registers), 2):
                bytes = struct.pack('>HH', rr.registers[i], rr.registers[i + 1])
                val = struct.unpack('>f', bytes)[0]
                values.append(val)
        
            # Publish MultiArray - Always with the same structure: [T, RH, Td, AH]
            msg = Float32MultiArray()
            msg.data = values
            self.multi_pub.publish(msg)

            # Publish in separate topics
            temperature, rel_humidity, dew_point, abs_humidity = values

            temp_msg = Temperature()
            temp_msg.header.stamp = self.get_clock().now().to_msg()
            temp_msg.temperature = temperature
            temp_msg.variance = 0.0
            self.temp_pub.publish(temp_msg)

            rh_msg = RelativeHumidity()
            rh_msg.header.stamp = self.get_clock().now().to_msg()
            rh_msg.relative_humidity = rel_humidity / 100.0  # RH as [0, 1] in sensor_msgs
            rh_msg.variance = 0.0
            self.rh_pub.publish(rh_msg)

            dew_msg = Temperature()
            dew_msg.header.stamp = self.get_clock().now().to_msg()
            dew_msg.temperature = dew_point
            self.dew_pub.publish(dew_msg)

            ah_msg = Temperature()
            ah_msg.header.stamp = self.get_clock().now().to_msg()
            ah_msg.temperature = abs_humidity
            self.ah_pub.publish(ah_msg)

            # dew_msg = Float32()
            # dew_msg.data = dew_point
            # self.dew_pub.publish(dew_msg)

            # ah_msg = Float32()
            # ah_msg.data = abs_humidity
            # self.ah_pub.publish(ah_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing registers: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PceP18Node()
    rclpy.spin(node)
    node.client.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# # One register at a time
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray, Float32
# from sensor_msgs.msg import Temperature, RelativeHumidity

# import struct
# from pymodbus.client import ModbusSerialClient

# class PceP18Node(Node):
#     def __init__(self):
#         super().__init__('pce_p18_node')

#         # Paramters
#         self.port = '/dev/ttyUSB1'      # needs to be changed!! (depends on computer assignment)
#         self.device_id = 1
#         self.timer_period = 5.0  # seconds

#         # Modbus register dictionary to read: {name: address} - adresses on datasheet table 7
#         self.register_map = {
#             'temperature': 7501,
#             'rel_humidity': 7502,
#             'dew_point': 7503,
#             'abs_humidity': 7504,
#         }

#         # Starting Modbus client
#         self.client = ModbusSerialClient(
#             port=self.port,
#             timeout=1,
#             baudrate=9600,
#             bytesize=8,
#             parity='N',
#             stopbits=1
#         )
#         self.client.connect()
#         self.get_logger().info("Modbus client created. Sensor already connected.")

#         # Publishers
#         self.multi_pub = self.create_publisher(Float32MultiArray, 'pce_p18/data', 10)
#         self.temp_pub = self.create_publisher(Temperature, 'pce_p18/temperature', 10)
#         self.rh_pub = self.create_publisher(RelativeHumidity, 'pce_p18/rel_humidity', 10)
#         self.dew_pub = self.create_publisher(Float32, 'pce_p18/dew_point', 10)
#         self.ah_pub = self.create_publisher(Float32, 'pce_p18/abs_humidity', 10)

#         # Timer
#         self.timer = self.create_timer(self.timer_period, self.read_sensor)

#     def read_sensor(self):
#         results = []
#         temp_val = None
#         rh_val = None
#         dew_val = None
#         ah_val = None

#         for name, address in self.register_map.items():
#             rr = self.client.read_holding_registers(address=address, count=2, device_id=self.device_id)
#             if rr.isError() or not hasattr(rr, 'registers'):
#                 self.get_logger().warn(f'Error reading register {address} ({name})')
#                 results.append(float('nan'))
#                 continue

#             try:
#                 bytes = struct.pack('>HH', rr.registers[0], rr.registers[1])
#                 value = struct.unpack('>f', bytes)[0]
#                 results.append(value)

#                 # Publish in separate topics
#                 if name == 'temperature':
#                     temp_val = value
#                 elif name == 'rel_humidity':
#                     rh_val = value
#                 if name == 'dew_point':
#                     dew_val = value
#                 elif name == 'abs_humidity':
#                     ah_val = value                    

#             except Exception as e:
#                 self.get_logger().error(f'Error when converting data from {name}: {e}')
#                 results.append(float('nan'))

#         # Publish MultiArray - Always with the same structure: [T, RH, Td, AH]
#         msg = Float32MultiArray()
#         msg.data = results
#         self.multi_pub.publish(msg)

#         # Publish in separate topics
#         if temp_val is not None:
#             temp_msg = Temperature()
#             temp_msg.temperature = temp_val
#             temp_msg.variance = 0.0
#             self.temp_pub.publish(temp_msg)

#         if rh_val is not None:
#             rh_msg = RelativeHumidity()
#             rh_msg.relative_humidity = rh_val / 100.0  # RH as [0, 1] in sensor_msgs
#             rh_msg.variance = 0.0
#             self.rh_pub.publish(rh_msg)

#         if dew_val is not None:
#             dew_msg = Float32()
#             dew_msg.data = dew_val
#             self.dew_pub.publish(dew_msg)

#         if ah_val is not None:
#             ah_msg = Float32()
#             ah_msg.data = ah_val
#             self.ah_pub.publish(ah_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = PceP18Node()
#     rclpy.spin(node)
#     node.client.close()
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
