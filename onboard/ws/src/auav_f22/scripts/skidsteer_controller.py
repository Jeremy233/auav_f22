#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from rover_control import compute_control
from rover_planning import RoverPlanner

def euler_from_quaternion(vals):
    x, y, z, w = vals
    sycp = 2 * (w * z + x * y)
    cycp = 1 - 2 * (y * y + z * z)
    return np.atan2(sycp, cycp)

class RoverController(Node):

    def __init__(self, positions):
        super().__init__('rover_controller')
        self.get_logger().info("init")
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_ready = self.create_subscription(Bool, '/ready', self.callback_ready, 1)
        self.pub_finished = self.create_publisher(Bool, '/finished', 10)
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.declare_parameter('~vehicle_frame', 'base_link')
        self.declare_parameter('~map_frame', 'map')
        self.declare_parameter('~delay', 10)
        self.declare_parameter('~v_max', 0.2)
        self.declare_parameter('~omega', 0.3)
        self.vehicle_frame = self.get_parameter('~vehicle_frame')
        self.map_frame = self.get_parameter('~map_frame')
        self.delay = self.get_parameter('~delay')  #  delay time
        self.v_max = self.get_parameter('~v_max')  #  max velocity
        self.omega_max = self.get_parameter('~omega')  #  max rotation rate
        self.ready = False
        self.positions = positions
        self.run()

    def __del__(self):
        self.stop()

    def callback_ready(self, msg):
        self.ready = msg.data
        

    def run(self):
        msg = Bool()
        msg.data = False
        self.pub_finished.publish(msg)

        # wait for drone ready
        rate = self.create_rate(1, self.get_clock())
        while rclpy.ok():
            self.get_logger().error('waiting for ready', throttle_duration_sec=10)
            rate.sleep()
            if self.ready:
                self.get_logger().info('trial is running')
                break

        # delay before start
        rclpy.sleep(self.delay)

        # run mode
        self.follow_reference()
        self.pub_finished.publish(True)
        self.get_logger().info('rover trajectory finished')

    def follow_reference(self):
        rate = self.create_rate(10)
        v = 0.3
        r = 0.5
        plot = False
        planner = RoverPlanner(x=0, y=2, v=v, theta=1.57, r=r)
        for position in self.positions:
            planner.goto(position[0], position[1], v, r)
        planner.stop(self.positions[-1][0], self.positions[-1][1])
        tf = np.sum(planner.leg_times)

        ref_data = planner.compute_ref_data(plot=plot)
        #if plot:
            #import matplotlib.pyplot as plt
            #plt.show()

        start = self.get_clock().now()
        while rclpy.ok():
            rate.sleep()
            # get transform from map to vehicle or stop after waiting 1 second
            try:
                trans = self.tfBuffer.lookup_transform(
                    target_frame=self.map_frame,
                    source_frame=self.vehicle_frame, time=self.get_clock().now(),
                    timeout=Duration(secs=1))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
                self.get_logger().error(e, throttle_duration_sec=10)
                self.stop()
                continue

            # calculate elapsed time
            t = (self.get_clock().now() - start).to_sec()
            if t > tf:
                break

            # get current SE2 pose
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            theta = euler_from_quaternion(
                    [trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w])

            v, omega = compute_control(t=t, x=x, y=y, theta=theta, ref_data=ref_data)
            # rospy.loginfo_throttle(1, 't: %g x: %g y: %g theta: %g v: %g omega: %g', t, x, y, theta, v, omega)

            # publish control
            self.move(v, omega)

    def move(self, v, omega):
        msg = Twist()
        msg.linear.x = float(v)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(omega)
        self.pub_cmd.publish(msg)

    def stop(self):
        self.get_logger().info("stop")
        self.move(v=0, omega=0)

def main(args=None):
    positions = [(0, 8), (-3, 8), (-3, 4), (0, 4), (0, 8), (-3, 8), (-3, 4), (0, 4)]
    #RoverController(positions)
    rclpy.init(args=args)
    rover_controller = RoverController(positions)
    rclpy.spin(rover_controller)
    rclpy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
	main()
