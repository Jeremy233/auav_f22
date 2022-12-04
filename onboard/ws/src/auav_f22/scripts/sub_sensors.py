#!/usr/bin/env python3
import rclpy
from px4_msgs.msg import SensorCombined
from rclpy.node import Node


class SensorInfoNode(Node):

	def __init__(self):
		super().__init__("sensor_subscriber")
		self.sensor_subscriber = self.create_subscription(SensorCombined, 
		'/SensorCombined_PubSubTopic', self.sensor_callback, 10)
		self.msg = None
		self.timestamp = None
		self.gyro_rad = None
		self.gyro_integral = None
		self.accelerometer_timestamp = None
		self.accelerometer_m_s2 = None
		self.accelerometer_integral = None
		self.accelerometer_clipping = None
		
	def sensor_callback(self, msg:SensorCombined):	
		self.msg = msg
		self.timestamp = msg.timestamp
		global timestamp
		timestamp = self.timestamp
		self.gyro_rad = msg.gyro_rad
		self.gyro_integral = msg.gyro_integral
		self.accelerometer_timestamp = msg.accelerometer_timestamp_relative + self.timestamp
		self.accelerometer_m_s2 = msg.accelerometer_m_s2
		self.accelerometer_integral = msg.accelerometer_integral_dt
		self.accelerometer_clipping = msg.accelerometer_clipping


def main(args=None):
	rclpy.init(args=args)
	sensor_info_node = SensorInfoNode()
	rclpy.spin(sensor_info_node)
	rclpy.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()