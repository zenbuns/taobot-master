#!/usr/bin/env python3

import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
import numpy as np
from smbus2 import SMBus
import my_publisher_node as mpn

    
class PidClass():
    def __init__(self):
        self.bus = SMBus(1)
        self.max_speed_normal = float(rospy.get_param("/maxvel",))
        self.max_speed_reduced = float(rospy.get_param("/maxvel", 0))
        self.transition_factor = 0
        self.delta_time = 0.05  # Adjust based on your needs
        self.is_sharp_turn = False
        self.sens_middle = 4.5  # Adjust based on your sensor setup
        self.integral = 0
        self.max_speed = self.max_speed_normal
        self.last_non_zero_error = 0

    def run(self, v_max, pid):
        if self.is_sharp_turn:
            # If a sharp turn is detected, reduce the maximum speed
            v_max = self.max_speed_reduced

        right_speed = max(0, v_max - pid)
        left_speed = max(0, v_max + pid)

        if right_speed < 0.05:
            left_speed = v_max + 0.15

        if left_speed < 0.05:
            right_speed = v_max + 0.15

        self.msg_wheels_cmd.vel_right = right_speed
        self.msg_wheels_cmd.vel_left = left_speed
        self.pub_wheels_cmd.publish(self.msg_wheels_cmd)

    def error_calculator(self, array_value):
        new_values_list = [1, 2, 3, 4, 5, 6, 7, 8]
        bitsum = 0
        counter = 0
        left_turn = False

        sharp_turn_array_value_list = [
            '10010000', '11001000', '10011000', '11001100',
            '01000100', '00100100', '01001000', '10001000', '01001100'
        ]

        if array_value in sharp_turn_array_value_list:
            self.is_sharp_turn = True
        else:
            self.is_sharp_turn = False

        for index in range(len(array_value)):
            if array_value[index] == "1":
                bitsum += new_values_list[index]
                counter += 1

        if counter == 0 or counter == 8:
            raise ValueError
        elif 3 <= counter <= 6:  # This elif calculates the error for 90c angles
            error = bitsum
        else:
            error = bitsum / counter
        return error, left_turn

    def pid_run(self, last_error, last_correction):
        while not rospy.is_shutdown():
            read = self.bus.read_byte_data(62, 17)
            read = bin(read)[2:].zfill(8)

            Kp = float(rospy.get_param("/p", 0.045))
            Ki = float(rospy.get_param("/i", 0.001))
            Kd = float(rospy.get_param("/d", 0.015))

            line_sens = []

            for indx, nr in enumerate(read):
                if nr == "1":
                    line_sens.append(indx + 1)

            if len(line_sens) > 0:
                error = self.sens_middle - np.average(line_sens)
                self.last_non_zero_error = error
            else:
                error = self.last_non_zero_error

            if self.is_sharp_turn:
                self.transition_factor = min(self.transition_factor + self.delta_time, 1)
            else:
                self.transition_factor = max(self.transition_factor - self.delta_time, 0)

            self.max_speed = (1 - self.transition_factor) * self.max_speed_normal + self.transition_factor * self.max_speed_reduced

            self.integral += (error + last_error) * self.delta_time / 2
            self.integral = min(max(self.integral, -2), 2)
            derivative = (error - last_error) / self.delta_time
            correction = Kp * error + Ki * self.integral + Kd * derivative

            last_error = error
            last_correction = correction

            return line_sens, correction, last_correction, last_error
