#! /usr/bin/env python

# import rospy
import pdb
import numpy as np
import math
from test_control import *
from test_get_jacobian import *


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
            for t in range(0, self.timespan, self.delta_t):
                index = int(t / self.delta_t)
                self.x[index], self.y[index], self.z[index], self.theta[index], self.phi[index], self.psi[
                    index] = self.get_instant_pose_helix(t)
        else:
            raise print('Wrong mode')
        return np.array([self.x, self.y, self.z, self.theta, self.phi, self.psi])


class KineControl:
    def __init__(self, desired_traj):
        self.num_joints = 7
        self.dsr_traj = desired_traj
        self.Q = np.zeros((self.num_joints, self.dsr_trj))
        self.dQ = np.zeros((self.num_joints, self.dsr_trj))
        self.ddQ = np.zeros((self.num_joints, self.dsr_trj))
        self.TAU = np.zeros((self.num_joints, self.dsr_trj))

        self.q0 = [0, math.pi / 3, 0, math.pi / 6, 0, 0, 0]
        self.q_dot0 = [0, 0, 0, 0, 0, 0, 0]
        self.pos0 = self.fk(self.q0)

    def fk(self, q):
        ...
        return fk

    def jacobian(self, q):
        jacobian_matrix = get_jacobian(q)
        return jacobian_matrix


    def control(self):
        K = 0.5 * np.array([[500, 0], [0, 500]])
        i = 1
        dt = 0.001
        ts = 3
        # q_r --> record the joint degrees
        # qd_r --> record the joint velocity
        q_r = np.zeros([7, 1])
        qd_r = np.zeros([7, 1])
        W = np.zeros([1, 1])
        # e_r --> record the endtip position error
        e_r = np.zeros([1, 1])
        # x_r --> record the real joint positions
        x1_r = np.zeros([7, 1])
        xtip_r = np.zeros([7, 1])

        # the initial state
        q = self.q0

        for t in range(0, self.dsr_traj.timespan, self.dsr_traj.delta_t):
            x_dsr, xd_dsr = self.dsr_traj.get_instant_pose_circle(t)

            # current state
            q_r = np.append(q_r, q, axis=1)
            X1, Xtip = self.fk(q)
            x1_r = np.append(x1_r, X1, axis=1)
            xtip_r = np.append(xtip_r, Xtip, axis=1)

            # control law
            e = x_dsr - Xtip
            J = self.jacobian(q)

            # manupilablity
            W = np.append(W, np.sqrt(np.linalg.det(J @ J.T)))

            # J_pseu = Jacobian_pseu(J)
            J_pseu = np.linalg.inv(J)

            qd_r = np.append(qd_r, J_pseu @ (xd_dsr + K @ e), axis=1)

            q = q + np.reshape(qd_r[:, i], (2, 1)) * dt

            # error
            e_r = np.append(e_r, np.sqrt(e.T @ e))
            i = i + 1

    def traj_vis(self):




if __name__ == '__main__':
    DsrTraj = DesiredTraj(0, 0, 0, 'circle', 10, 0.001, 0.1, 10, 10)
    # desired_traj = Traj.get_desired_pose()

    kinecontroller = KineControl(DsrTraj)
    kinecontroller.control()
    kinecontroller.traj_vis()
