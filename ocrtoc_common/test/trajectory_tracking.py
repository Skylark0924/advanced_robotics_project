#! /usr/bin/env python

import pdb
import numpy as np
import math
from test_control import *


class DesiredTraj:
    def __init__(self, timespan, delta_t, r=None, num=None, shift=None):
        self.x, self.y, self.z, self.theta, self.phi, self.psi = np.zeros(timespan), np.zeros(timespan), np.zeros(
            timespan), np.zeros(timespan), np.zeros(timespan), np.zeros(timespan)
        self.xt, self.yt, self.zt, self.thetat, self.phit, self.psit = 0.125, 0.125, 0.5, 0, 0, 0

        self.r = r
        self.num = num
        self.shift = shift
        self.timespan = timespan
        self.delta_t = delta_t
        self.theta_circle = 0

    def get_instant_pose_circle(self):
        vy = 0
        vz = 0.5
        vth = 7
        delta_y = (vy * math.cos(self.theta_circle) - vz * math.sin(self.theta_circle)) * self.delta_t
        delta_z = (vy * math.sin(self.theta_circle) + vz * math.cos(self.theta_circle)) * self.delta_t
        delta_th = vth * self.delta_t
        self.yt += delta_y
        self.zt += delta_z
        self.theta_circle += delta_th

        traj_t = np.array([self.xt, self.yt, self.zt, self.thetat, self.phit, self.psit])
        trajd_t = np.array([0, 0, 0, 0, 0, 0])
        return traj_t, trajd_t


class KineControl:
    def __init__(self, desired_traj):
        self.num_joints = 7
        self.dsr_traj = desired_traj

        self.q0 = np.array([0.88, -0.22, -0.19, -1.41, -0.645, 1.51, 0.08])
        self.q_dot0 = [0, 0, 0, 0, 0, 0, 0]

    def fk(self, q):
        """
        Need to be implemented
        You can use the 'get_fk' function I provided to verify your results, but do not use it for your project.
        """
        pose_value = ...
        return pose_value

    def jacobian(self, q):
        """
        Need to be implemented
        You can use the 'get_jacobian' function I provided to verify your results, but do not use it for your project.
        """
        jacobian_matrix = ...
        return jacobian_matrix

    def control(self):
        """
        Need to be implemented
        """
        dt = self.dsr_traj.delta_t
        ts = self.dsr_traj.timespan

        ...

        for t in np.arange(0, ts, dt).reshape(-1):
            print('=' * 80)

            x_dsr, xd_dsr = self.dsr_traj.get_instant_pose_circle(t)

            ...

        # convert the whole trajectory with a shape of [ts/dt, 7]
        # into a 1-dim array [ts/dt * 7, ] by using '.reshape((-1,))'
        joint_goal_lst = ...

        execute_joint_traj_goal(joint_goal_lst)

    def traj_vis(self):
        """
        Need to be implemented
        Plot and save the following figures:
        1. tracking_error(time, e_r)
        2. joint_angles(time, q_r)
        3. joint_velocities(time, qd_r)
        4. manipulability(time,W)
        """
        ...


if __name__ == '__main__':
    # You can define any trajectory you like, there is no limit.
    # If you define a new trajectory, the visualization script 'test_dsr_traj_vis.py' need to modify correspondingly.
    DsrTraj = DesiredTraj(timespan=3, delta_t=0.005, r=1, num=10, shift=10)

    kinecontroller = KineControl(DsrTraj)
    kinecontroller.control()
    kinecontroller.traj_vis()
