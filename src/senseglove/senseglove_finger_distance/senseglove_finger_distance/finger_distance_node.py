# Copyright 2022 Florent AUDONNET

from math import pow, sqrt

import rclpy
from finger_distance_calibration import Calibration
from rclpy.node import Node
from senseglove_shared_resources.msg import (FingerDistanceFloats,
                                             SenseGloveState)
from senseglove_shared_resources.srv import PinchCalibration


class FingerTipHandler(Node):
    handedness_list = ["/lh", "/rh"]

    def __init__(self, glove_nr="1", calib_mode='nothing', finger_nrs=[3, 7, 11, 15, 19]):
        super().__init__('senseglove_finger_distance_node')
        self.finger_nrs = finger_nrs
        self.calib_mode = calib_mode
        self.finger_tips = [FingerTipVector() for i in self.finger_nrs]
        self.senseglove_ns = "senseglove/" + str(self.handedness_list[int(glove_nr) % 2])
        self.create_subscription(SenseGloveState, self.senseglove_ns + "/senseglove_states", self.callback, 10)
        self.pub = self.create_publisher(FingerDistanceFloats, self.senseglove_ns + "/finger_distances" , 10)
        self.pinch_min_cli = self.create_client(PinchCalibration, f"{self.senseglove_ns}/pinch_calibration_min")
        self.pinch_max_cli = self.create_client(PinchCalibration, f"{self.senseglove_ns}/pinch_calibration_max")

        self.calibration = Calibration("default")

    def apply_calib(self, pinch_value=0.0, pinch_combination=0, mode='nothing'):
        if mode == 'nothing':
            return pinch_value
        if mode == 'minimum':
            # Return the values so that pinching your fingers results in a finger distance of zero
            return pinch_value - self.calibration.pinch_calibration_min[pinch_combination]
        elif mode == 'normalized':
            # Return normalized finger distance value between 0 and 1
            return (pinch_value - self.calibration.pinch_calibration_min[pinch_combination]) / \
                   self.calibration.pinch_calibration_max[pinch_combination]

    def distance_publish(self):
        finger_distance_message = FingerDistanceFloats()
        finger_distance_message.th_ff.data = self.apply_calib((self.finger_tips[0] - self.finger_tips[1]).magnitude(),
                                                              0, self.calib_mode)
        finger_distance_message.th_mf.data = self.apply_calib((self.finger_tips[0] - self.finger_tips[2]).magnitude(),
                                                              1, self.calib_mode)
        finger_distance_message.th_rf.data = self.apply_calib((self.finger_tips[0] - self.finger_tips[3]).magnitude(),
                                                              2, self.calib_mode)
        finger_distance_message.th_lf.data = (self.finger_tips[0] - self.finger_tips[4]).magnitude()
        self.pub.publish(finger_distance_message)

    def make_service_request(self, service):
        while not service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = PinchCalibration.Request()
        future = service.call_async(self.req)
        return future

    def get_service_result(self, future):
        if future.done():
            try:
                response = future.result()
            except Exception as e:
                self.get_logger().info(
                    'Service call failed %r' % (e,))
                return []
            return response.calibration


    def callback(self, data):
        if not self.calibration.is_calibrated():
            min_future = self.get_service_result(self.make_service_request(self.pinch_min_cli))
            max_future = self.get_service_result(self.make_service_request(self.pinch_max_cli))

            if max_future != [] and min_future != []:
                self.calibration = Calibration("from_param_server")
                if min_future != []:
                    self.pinch_calibration_min = min_future  # [index, middle, ring][x, y, z] random values from Kees
                if max_future != []:
                    self.pinch_calibration_max = max_future  # [index, middle, ring] in mm
            else:
                self.get_logger().warning("No calibration data found, using defaults")

        for i in range(len(self.finger_nrs)):
            self.finger_tips[i].x = data.finger_tip_positions[i].x
            self.finger_tips[i].y = data.finger_tip_positions[i].y
            self.finger_tips[i].z = data.finger_tip_positions[i].z
        self.distance_publish()


class FingerTipVector:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, other):
        return FingerTipVector(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return FingerTipVector(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, other):
        return self.x * other.x + self.y * other.y + self.z * other.z

    def magnitude(self):
        return sqrt(pow(self.x, 2) + pow(self.y, 2) + pow(self.z, 2))


def main(glove_nr, calib_mode, args=None):
    rclpy.init(args=args)
    node = FingerTipHandler(glove_nr=glove_nr, calib_mode=calib_mode)

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()