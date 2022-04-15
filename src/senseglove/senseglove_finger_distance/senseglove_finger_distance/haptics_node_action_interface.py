import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import sys
from std_msgs.msg import Header
import numpy as np

from copy import copy
from time import time

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from threading import Event, Thread

from rclpy.duration import Duration

class Trajectory(Node):
    def __init__(self, ns='', joint_names=['empty'], goal_time_tol=0.01, timeout=0.001):
        super().__init__('senseglove_haptics_node')

        self.wait_for_goal_timeout = timeout
        self._joint_names = joint_names

        self._client = ActionClient(self,
                                    FollowJointTrajectory,
                                    ns + "follow_joint_trajectory",
                                    )
        self._goal_time_tolerance = Duration(seconds=goal_time_tol).to_msg()
        self.amp = 50  # percentage
        self.wave = np.linspace(0, np.pi * 10, 201)
        self.rand_traj_points = [0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0]  # what you will!

    def send_goal(self):
        if not self._client.wait_for_server(timeout=rclpy.Duration(10.0)):
            self.get_logger().info("Timed out waiting for Joint Trajectory"
                                   " Action Server to connect. Start the action server"
                                   " before running example.")
            return
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.goal_time_tolerance = self._goal_time_tolerance
        goal_msg.trajectory.joint_names = self._joint_names
        for i in range(200):
            self.rand_traj_points[0] = self.amp * \
                np.sin(self.wave[i]) + self.amp
            self.add_point(goal_msg, self.rand_traj_points, 0.01)
            self._send_goal_future = self._client.send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(
                self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))

    def add_point(self, goal_msg, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rclpy.Duration(time)
        goal_msg.trajectory.points.append(point)


def infinity_loop(traj: Trajectory, condition: Event):
    while not condition.is_set():
        traj.send_goal()


def main(args=None):
    rclpy.init(args=args)
    condition = Event()

    joint_list = ["thumb_brake", " index_brake", "middle_brake", "ring_brake", "pinky_brake", "thumb_cmc", "index_mcp",
                  "middle_mcp", "ring_mcp", " pinky_mcp"]
    # ns = '/senseglove/lh'
    ns = ''
    action_ns = ns + '/joint_trajectory_controller/'

    traj = Trajectory(ns=action_ns, joint_names=joint_list,
                      goal_time_tol=1.0, timeout=0.001)
    executor = rclpy.executors.MultiThreadedExecutor()

    thread = Thread(target=infinity_loop, args=(traj,))
    rclpy.spin(traj, executor=executor)
    condition.set()  # End while loop.
    thread.join()
    traj.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
