#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# import pyquaternion
from hippo_control_msgs.msg import ActuatorSetpoint
from nav_msgs.msg import Odometry
import tf_transformations
import math
from hippo_msgs.msg import DepthStamped
from sensor_msgs.msg import FluidPressure

from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


class RollPitchStabilizerNode(Node):

    def __init__(self, node_name):
        super().__init__(node_name=node_name)

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST,
                         depth=1)

        self.torque_pub = self.create_publisher(ActuatorSetpoint,
                                                'torque_setpoint',
                                                qos_profile=1)
        self.thrust_pub = self.create_publisher(ActuatorSetpoint,
                                                'thrust_setpoint',
                                                qos_profile=1)
        self.attitude_sub = self.create_subscription(Odometry,
                                                     'odometry',
                                                     self.on_attitude,
                                                     qos_profile=1)
        self.pressure_sub = self.create_subscription(FluidPressure,
                                                     'pressure',
                                                     self.on_pressure,
                                                     qos_profile=qos)

    def on_attitude(self, msg: Odometry):
        q = msg.pose.pose.orientation
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])
        timestamp = rclpy.time.Time.from_msg(msg.header.stamp)
        roll_ctrl_output = self.control_roll(roll)
        pitch_ctrl_output = self.control_pitch(pitch)

        self.publish_control_output(roll_ctrl_output, pitch_ctrl_output,
                                    timestamp)

    def on_pressure(self, msg: FluidPressure):
        pressure = msg.fluid_pressure

        depth = self.pressure_to_depth(pressure)
        timestamp = rclpy.time.Time.from_msg(msg.header.stamp)
        depth_ctrl_output = self.control_depth(depth)
        self.publish_thrust_control_output(depth_ctrl_output, timestamp)

    def pressure_to_depth(self, pressure: float):
        atmospheric_pressure = 100000.0
        depth = -(pressure - atmospheric_pressure) / 10000
        # self.get_logger().info(f'Depth: {depth}')
        return depth

    def wrap_pi(self, value: float):
        """Normalize the angle to the range [-pi, pi]."""
        if (-math.pi < value) and (value < math.pi):
            return value
        range_ = 2 * math.pi
        num_wraps = math.floor((value + math.pi) / range_)
        return value - range_ * num_wraps

    def control_depth(self, depth):
        # setpoint = -0.43
        setpoint = -0.25
        error = setpoint - depth
        p_gain = 100.0
        return p_gain * error

    def control_roll(self, depth):
        setpoint = 0.0
        error = self.wrap_pi(setpoint - depth)
        p_gain = 8.0
        return p_gain * error

    def control_pitch(self, pitch):
        setpoint = 0.0
        error = self.wrap_pi(setpoint - pitch)
        p_gain = 5.0
        return p_gain * error

    def publish_control_output(self, control_output_roll: float,
                               control_output_pitch: float,
                               timestamp: rclpy.time.Time):

        msg = ActuatorSetpoint()
        msg.header.stamp = timestamp.to_msg()
        msg.ignore_x = False
        msg.ignore_y = False
        msg.ignore_z = True

        msg.x = control_output_roll
        msg.y = control_output_pitch
        self.torque_pub.publish(msg)

    def publish_thrust_control_output(self, control_output_depth: float,
                                      timestamp: rclpy.time.Time):

        msg = ActuatorSetpoint()
        msg.header.stamp = timestamp.to_msg()
        msg.ignore_x = True
        msg.ignore_y = True
        msg.ignore_z = False

        msg.z = control_output_depth
        self.thrust_pub.publish(msg)


def main():
    rclpy.init()
    node = RollPitchStabilizerNode("stabilizer_node")
    rclpy.spin(node)


if __name__ == "__main__":
    main()
