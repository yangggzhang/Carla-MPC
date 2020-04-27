#!/usr/bin/env python3
import numpy as np
from numpy import linalg

import cvxpy

from matplotlib import pyplot as plt

class MPC:
    def __init__(self, x=0, y=0, yaw=0, v=0, delta=0,
                 max_steering_angle=1.22, L=3, lf=1.5, lr=1.5, Q=np.eye(4), Qf=np.eye(4),
                 R=np.eye(2), Rd=np.eye(2), len_horizon=5, a_max=2, a_min=-1,
                 a_rate_max=1, steer_rate_max=0.5, v_min=-1, v_max=80, dist = 1.5, time_step = 0.1):

        # States
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

        # Steering angle
        self.delta = delta
        self.max_steering_angle = max_steering_angle

        # Wheel base
        self.L = L
        self.lf = lf
        self.lr = lr

        # Control gain
        self.Q = Q
        self.R = R
        self.Rd = Rd
        self.Qf = Qf

        #forward time step
        self.time_step = time_step

        self.len_horizon = len_horizon

        self.v_min = v_min
        self.v_max = v_max
        self.a_max = a_max
        self.a_min = a_min
        self.a_rate_max = a_rate_max
        self.steer_rate_max = steer_rate_max
        self.dist = dist

        self.prev_idx = 0
        self.send_prev = 0
        self.prev_accelerations = np.array([0.0] * self.len_horizon)
        self.prev_deltas = np.array([0.0] * self.len_horizon)
        self.prev_index = 0

    def update_position(self, x, y):
        self.x = x
        self.y = y

    def update_speed(self, v):
        self.v = v

    def update_yaw(self, yaw):
        self.yaw = yaw

    @staticmethod
    def bound_angles(theta):
        if theta > np.pi:
            theta = theta - 2*np.pi
        elif theta < -np.pi:
            theta = theta + 2*np.pi

        return theta

    def find_closest_waypoint(self, cx, cy):
        distances = np.sum(( np.array([[self.x], [self.y]]) -
                             np.stack((cx, cy)) )**2, axis=0)
        idx = np.argmin(distances)

        return idx, cx[idx], cy[idx]

    def get_linearized_dynamics(self, psi, delta, v, dt=0.01):
        self.lr = 0.5 * self.L
        A = np.array([
                     [ 1, 0,                         np.cos(psi + np.arctan((self.lr*np.tan(delta))/(self.L))), -v*np.sin(psi + np.arctan((self.lr*np.tan(delta))/(self.L)))],
                     [ 0, 1,                         np.sin(psi + np.arctan((self.lr*np.tan(delta))/(self.L))),  v*np.cos(psi + np.arctan((self.lr*np.tan(delta))/(self.L)))],
                     [ 0, 0,                                                                  1,                                             0],
                     [ 0, 0, np.tan(delta)/((self.L)*np.sqrt((self.lr**2*np.tan(delta)**2)/(self.L)**2 + 1)),                                             1]
                     ])

        B = np.array([
                     [ 0,                                                             -(self.lr*v*np.sin(psi + np.arctan((self.lr*np.tan(delta))/(self.L)))*(np.tan(delta)**2 + 1))/((self.L)*((self.lr**2*np.tan(delta)**2)/(self.L)**2 + 1))],
                     [ 0,                                                              (self.lr*v*np.cos(psi + np.arctan((self.lr*np.tan(delta))/(self.L)))*(np.tan(delta)**2 + 1))/((self.L)*((self.lr**2*np.tan(delta)**2)/(self.L)**2 + 1))],
                     [ 1,                                                                                                                                                                                   0],
                     [ 0, (v*(np.tan(delta)**2 + 1))/((self.L)*np.sqrt((self.lr**2*np.tan(delta)**2)/(self.L)**2 + 1)) - (self.lr**2*v*np.tan(delta)**2*(np.tan(delta)**2 + 1))/((self.L)**3*np.sqrt((self.lr**2*np.tan(delta)**2)/(self.L)**2 + 1)**3)]
                    ])

        A[0, 2] *= dt
        A[0, 3] *= dt
        A[1, 2] *= dt
        A[1, 3] *= dt
        A[3, 2] *= dt

        B[0, 1] *= dt
        B[1, 1] *= dt
        B[2, 0] *= dt
        B[3, 1] *= dt
        return A, B
        # C = np.zeros((4, 1))
        # C[0, 0] = v * np.sin(yaw) * yaw * dt
        # C[1, 0] = - v * np.cos(yaw) * yaw * dt
        # C[3, 0] = - v * delta * dt / (self.L * np.cos(delta)**2)
        

    def linear_mpc(self, z_ref, z_initial, prev_deltas, dt):
        z = cvxpy.Variable((4, self.len_horizon + 1))
        u = cvxpy.Variable((2, self.len_horizon))

        cost = 0
        constraints = [z[:, 0] == z_initial.flatten()]
        for i in range(self.len_horizon - 1):
            ## Cost
            if i != 0:
                cost += cvxpy.quad_form(z_ref[:, i] - z[:, i], self.Q)
                cost += cvxpy.quad_form(u[:, i] - u[:, i-1], self.Rd)
            else:
                u_prev = [self.prev_accelerations[0], self.prev_deltas[0]]
                cost += cvxpy.quad_form(u[:, i] - u_prev, self.Rd)

            cost += cvxpy.quad_form(u[:, i], self.R)

            ## Constraints
            A, B = self.get_linearized_dynamics(z_ref[3, i], self.prev_deltas[np.min([ i + 1, len(self.prev_deltas) - 1])],
                                                   z_ref[2, i], dt)
            constraints += [z[:, i+1] == A @ z[:, i] + B @ u[:, i]]

            # Velocity limits
            constraints += [z[2, i] <= self.v_max]
            constraints += [z[2, i] >= self.v_min]

            # Input limits
            constraints += [self.a_min <= u[0, i]]
            constraints += [u[0, i] <= self.a_max]
            constraints += [u[1, i] <= self.max_steering_angle]
            constraints += [u[1, i] >= -self.max_steering_angle]
            constraints += [(z[0, i] - z_ref[0, i])*np.sin(z_ref[3,i]) <= self.dist]
            constraints += [(z[0, i] - z_ref[0, i])*np.sin(z_ref[3,i]) >= -self.dist]
            constraints += [(z[1, i] - z_ref[1, i])*np.cos(z_ref[3,i]) <= self.dist]
            constraints += [(z[1, i] - z_ref[1, i])*np.cos(z_ref[3,i]) >= -self.dist]

            # Rate of change of input limit
            if i != 0:
                constraints += [u[0, i] - u[0, i-1] <= self.a_rate_max]
                constraints += [u[0, i] - u[0, i-1] >= -self.a_rate_max]
                # constraints += [u[1, i] - u[1, i-1] <= self.steer_rate_max * dt]
                # constraints += [u[1, i] - u[1, i-1] >= -self.steer_rate_max * dt]

        # Terminal cost
        cost += cvxpy.quad_form(z_ref[:, -1] - \
                                z[:, -1], self.Qf)

        # Quadratic Program
        qp = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        qp.solve(solver=cvxpy.ECOS, verbose=False)

        if qp.status == cvxpy.OPTIMAL or qp.status == cvxpy.OPTIMAL_INACCURATE:
            x = np.array(z.value[0, :]).flatten()
            y = np.array(z.value[1, :]).flatten()
            v = np.array(z.value[2, :]).flatten()
            yaw = np.array(z.value[3, :]).flatten()
            a = np.array(u.value[0, :]).flatten()
            delta = np.array(u.value[1, :]).flatten()
        else:
            # x, y, v, yaw, a, delta = None, None, None, None, None, None
            print("No results")
            a, delta = None, None

        return a, delta

    def get_ref_traj(self, cx, cy, cyaw, ck, vel, prev_idx, dt=0.01):
        x_ref = np.zeros((4, self.len_horizon+1))
        idx, target_x, target_y = self.find_closest_waypoint(cx, cy)
        if idx <= prev_idx:
            idx = prev_idx
            target_x = cx[idx]
            target_y = cy[idx]

        x_ref[0, 0] = cx[idx]
        x_ref[1, 0] = cy[idx]
        x_ref[2, 0] = vel[idx]
        x_ref[3, 0] = cyaw[idx]

        idxs = [idx]
        path_length = 0
        next_idx = idx
        for i in range(self.len_horizon):
            path_length += abs(vel[idx]) * dt
            while next_idx < len(cx):
                if (cx[idx] - cx[next_idx])**2 + (cy[idx] - cy[next_idx])**2 > path_length**2:
                    x_ref[0, i+1] = cx[next_idx]
                    x_ref[1, i+1] = cy[next_idx]
                    x_ref[2, i+1] = vel[next_idx]
                    x_ref[3, i+1] = cyaw[next_idx]
                    idxs.append(next_idx)
                    next_idx += 1
                    break
                next_idx += 1
            if next_idx == len(cx):
                x_ref[0, i+1] = cx[-1]
                x_ref[1, i+1] = cy[-1]
                x_ref[2, i+1] = vel[-1]
                x_ref[3, i+1] = cyaw[-1]
                idxs.append(len(cx)-1)

        return idxs, x_ref

    def get_inputs(self, x, y, yaw, v, waypoints):
        self.update_position(x, y)
        self.update_yaw(yaw)
        self.update_speed(v)

        x0 = np.array([[x], [y], [v], [yaw]])

        accelerations, deltas = self.linear_mpc(waypoints, x0, self.prev_deltas, dt=self.time_step)

        if accelerations is None:
            self.prev_accelerations = self.prev_accelerations[1:]
            self.prev_deltas = self.prev_deltas[1:]
        else:
            self.prev_accelerations = accelerations
            self.prev_deltas = deltas

        return self.prev_accelerations[0], self.prev_deltas[0]