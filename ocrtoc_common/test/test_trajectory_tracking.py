#! /usr/bin/env python

# import rospy
import pdb
import numpy as np
import math
from test_control import *


# from ocrtoc_common.srv import *


class DesiredTraj:
    def __init__(self, x0, y0, z0, mode, timespan, delta_t, r=None, num=None, shift=None):
        self.x, self.y, self.z, self.theta, self.phi, self.psi = np.zeros(timespan), np.zeros(timespan), np.zeros(
            timespan), np.zeros(timespan), np.zeros(timespan), np.zeros(timespan)
        self.x0, self.y0, self.z0, self.theta0, self.phi0, self.psi0 = x0, y0, z0, 0, 0, 0

        self.mode = mode
        self.r = r
        self.num = num
        self.shift = shift
        self.timespan = timespan
        self.delta_t = delta_t

    def get_instant_pose_circle(self, t):
        x = self.x0 + self.r * math.cos(t * 2 * math.pi)
        y = self.y0
        z = self.z0 + self.r * math.sin(t * 2 * math.pi)
        theta = 0.1 * math.sin(t * 2 * math.pi)
        phi = self.phi0
        psi = self.psi0
        return x, y, z, theta, phi, psi

    def get_instant_pose_helix(self, t):
        x = self.x0 + self.r * math.cos(t * self.num * 2 * math.pi)
        y = self.y0 + t * self.num * self.shift
        z = self.z0 + self.r * math.sin(t * self.num * 2 * math.pi)
        theta = self.theta0
        phi = self.phi0
        psi = self.psi0
        return x, y, z, theta, phi, psi

    def get_desired_pose(self):
        if self.mode == 'circle':
            for t in range(0, self.timespan, self.delta_t):
                index = int(t / self.delta_t)
                self.x[index], self.y[index], self.z[index], self.theta[index], self.phi[index], self.psi[
                    index] = self.get_instant_pose_circle(t)
        elif self.mode == 'helix':
            for t in range(self.timespan):
                index = int(t / self.delta_t)
                self.x[index], self.y[index], self.z[index], self.theta[index], self.phi[index], self.psi[
                    index] = self.get_instant_pose_helix(t)
        else:
            raise print('Wrong mode')
        return np.array([self.x, self.y, self.z, self.theta, self.phi, self.psi])



if __name__ == '__main__':
    Traj = DesiredTraj(0, 0, 0, 'circle', 10, 0.001, 0.1, 10, 10)
    desired_traj = Traj.get_desired_pose()
