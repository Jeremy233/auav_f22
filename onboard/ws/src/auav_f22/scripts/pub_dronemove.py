#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, Timesync, OffboardControlMode

class DroneMoveNode(Node):

    def __init__(self):
        super().__init__("drone_movement")
        self.time_subscriber = self.create_subscription(Timesync, "/Timesync_PubSubTopic", self.time_callback, 10)
        
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, 'OffboardControlMode_PubSubTopic', 10)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, 'TrajectorySetpoint_PubSubTopic', 10)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, 'VehicleCommand_PubSubTopic', 10)
        self.runner = self.create_timer(.1, self.runner_callback)
        self.setpoint = 0
        self.time = 0


    def runner_callback(self):
        if self.setpoint == 10:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

            self.arm()
        
            self.get_logger().info('armed drone')

        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()
        self.get_logger().info('time: "%d"' % self.time)

        if self.setpoint < 11:
            self.setpoint += 1
        
    
    def time_callback(self, msg:Timesync):
        self.time = msg.timestamp

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.time
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_control_mode_publisher.publish(msg)


    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        # arm.timestamp = self.get_clock().now()
        # timestamp = self.get_clock().now()
        msg.timestamp = self.time
        # print(timestamp)
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher.publish(msg)
        
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = self.time
        msg.x = 0.0
        msg.y = 0.0
        msg.z = -5.0
        msg.yaw = -3.14
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info("123")

def main(args=None):
    rclpy.init(args=args)
    dronemovement = DroneMoveNode()
    rclpy.spin(dronemovement)
    dronemovement.distroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()