#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""
import cutils
import numpy as np
from matplotlib import pyplot as plt
from Controller.MPC import MPC
from Controller.LongitudinalPID import LongitudinalPID

# class LongitudinalPID:
#     """
#     PID controller for longitudinal control
#     """
#     def __init__(self, v=0, L=3, Kp=1, Kd=0.01, Ki=0.01,
#                  integrator_min=None, integrator_max=None):
#         # States
#         self.v = v
#         self.prev_error = 0
#         self.sum_error = 0

#         # Wheel base
#         self.L = L

#         # Control gain
#         self.Kp = Kp
#         self.Ki = Ki
#         self.Kd = Kd

#         self.integrator_min = integrator_min
#         self.integrator_max = integrator_max

#     def update_speed(self, v):
#         self.v = v

#     def get_throttle_input(self, v, dt, target_speed):
#         self.update_speed(v)

#         error = target_speed - self.v
#         self.sum_error += error * dt
#         if self.integrator_min is not None:
#             self.sum_error = np.fmax(self.sum_error,
#                                      self.integrator_min)
#         if self.integrator_max is not None:
#             self.sum_error = np.fmin(self.sum_error,
#                                      self.integrator_max)

#         throttle = self.Kp * error + \
#             self.Ki * self.sum_error + \
#             self.Kd * (error - self.prev_error) / dt
#         self.prev_error = error

#         return throttle


class Controller(object):
    def __init__(self, waypoints, controller_type="MPC"):
        self.vars = cutils.CUtils()
        self._lookahead_distance = 3.0
        self._lookahead_time = 1.0
        self._current_x = 0
        self._current_y = 0
        self._current_yaw = 0
        self._current_speed = 0
        self._desired_speed = 0
        self._current_frame = 0
        self._current_timestamp = 0
        self._start_control_loop = False
        self._set_throttle = 0
        self._set_brake = 0
        self._set_steer = 0
        self._waypoints = waypoints
        self._conv_rad_to_steer = 180.0 / 70.0 / np.pi

        self.longitudinal_controller = LongitudinalPID(self._current_speed,
                                                       Kp=1.0,
                                                       Kd=0.1,
                                                       Ki=0.1,
                                                       integrator_max=10.0,
                                                       integrator_min=-10.0)
        Q = np.eye(4)
        R = 0.01*np.eye(2)
        Qf = 5*np.eye(4)
        Rd = np.eye(2)
        self.controller = MPC(x=self._current_x, y=self._current_y, yaw=self._current_yaw,
                                v=self._current_speed, delta=0,
                                L=2, Q=Q, R=R, Qf=Qf, Rd=Rd,
                                len_horizon=10)

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x = x
        self._current_y = y
        self._current_yaw = yaw
        self._current_speed = speed
        self._current_timestamp = timestamp
        self._current_frame = frame
        if self._current_frame:
            self._start_control_loop = True

    def get_lookahead_index(self, lookahead_distance):
        min_idx = 0
        min_dist = float("inf")
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                self._waypoints[i][0] - self._current_x,
                self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i

        total_dist = min_dist
        lookahead_idx = min_idx
        for i in range(min_idx + 1, len(self._waypoints)):
            if total_dist >= lookahead_distance:
                break
            total_dist += np.linalg.norm(np.array([
                self._waypoints[i][0] - self._waypoints[i-1][0],
                self._waypoints[i][1] - self._waypoints[i-1][1]]))
            lookahead_idx = i
        return lookahead_idx

    def update_desired_speed(self):
        min_idx = 0
        min_dist = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                self._waypoints[i][0] - self._current_x,
                self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        self._desired_speed = self._waypoints[min_idx][2]

    def smooth_yaw(self, yaws):
        for i in range(len(yaws) - 1):
            dyaw = yaws[i+1] - yaws[i]

            while dyaw >= np.pi/2.0:
                yaws[i+1] -= 2.0 * np.pi
                dyaw = yaws[i+1] - yaws[i]

            while dyaw <= -np.pi/2.0:
                yaws[i+1] += 2.0 * np.pi
                dyaw = yaws[i+1] - yaws[i]

        return yaws

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x = self._current_x
        y = self._current_y
        yaw = self._current_yaw
        v = self._current_speed
        self.update_desired_speed()
        v_desired = self._desired_speed
        t = self._current_timestamp
        waypoints = self._waypoints
        throttle_output = 0
        steer_output = 0
        brake_output = 0

        self.vars.create_var('t_prev', 0.0)

        # Skip the first frame to store previous values properly
        if self._start_control_loop:

            dt = t - self.vars.t_prev
            throttle_output = self.longitudinal_controller.get_throttle_input(
                v, dt, v_desired)

            cyaw = [yaw]
            cx = []
            cy = []
            speed_profile = []
            for i in range(len(self._waypoints)-1):
                cyaw.append(np.arctan2(self._waypoints[i+1][1] - self._waypoints[i][1],
                                        self._waypoints[i+1][0] - self._waypoints[i][0]))
                cx.append(self._waypoints[i][0])
                cy.append(self._waypoints[i][1])
                speed_profile.append(self._waypoints[i][2])

            cyaw.append(cyaw[-1])
            cx.append(self._waypoints[-1][0])
            cy.append(self._waypoints[-1][1])
            speed_profile.append(self._waypoints[-1][2])
            ck = [0.0] * len(self._waypoints)

            cyaw = self.smooth_yaw(cyaw)
            del cyaw[0]

            # plt.figure(0)
            # plt.cla()
            # plt.plot(cx, cy, '-c')

            acceleration, steer_output, xs, ys, vs, yaws = \
                self.controller.get_inputs(x, y, yaw,
                                            v, np.stack((cx, cy, cyaw, ck)),
                                            speed_profile,
                                            0.1)

            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
        self.set_throttle(throttle_output)  # in percent (0 to 1)
        self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
        self.set_brake(brake_output)        # in percent (0 to 1)
        self.vars.t_prev = t
