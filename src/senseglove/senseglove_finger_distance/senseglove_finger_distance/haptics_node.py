# Copyright 2022 Florent AUDONNET

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray

class SenseGloveHaptics(Node):

    def __init__(self):
        super().__init__('senseglove_haptics_node')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/senseglove/lh/joint_position_controller/commands', 10)
        self.get_logger().info('Initialize haptics node')
        self.joint_list = ['empty']
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float64MultiArray()
        # This will set the first 5 joints aka the brakes to their most restrictive value and will activate the last 5 joints to vibrate at their biggest intensity.
        msg.data = [100,100,100,100,100,100,100,100,100,100]
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    haptics = SenseGloveHaptics()
    rclpy.spin(haptics)

    rclpy.shutdown()
