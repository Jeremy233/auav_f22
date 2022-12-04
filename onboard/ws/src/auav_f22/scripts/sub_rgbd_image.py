#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import time
import os.path
import numpy as np
# from multiprocessing import Process, Pipe
import pickle

class RGBDImageNode(Node):

	def __init__(self):
		super().__init__("RGBDImage_subscriber")
		self.RGBDImage_subscriber = self.create_subscription(Image, 
		"/rgbd_camera/depth_image", self.sensor_callback, 10)
		self.height = None
		self.width = None
		self.encoding = None
		self.data = None
		self.rgbd_image = None

	def sensor_callback(self, msg:Image):
		self.height = msg.height
		self.width = msg.width
		self.encoding = msg.encoding
		self.data = msg.data
		# global set_rgbd_image
		self.rgbd_image = np.array(self.data).reshape(480, 640, 4)
		# set_rgbd_image = self.rgbd_image
		shared = self.rgbd_image
		fp = open("shared.npy", "wb")
		# pickle.dump(shared, fp)
		np.save(fp, shared)
		self.get_logger().info(str(self.rgbd_image.shape))

def main(args=None):
	rclpy.init(args=args)
	rgbd_image_node = RGBDImageNode()
	rclpy.spin(rgbd_image_node)
	rclpy.shutdown()

# def f(child_conn):
# 	msg = set_rgbd_image
# 	child_conn.send(msg)
# 	child_conn.close()

if __name__ == '__main__':
	main()