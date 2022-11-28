#! /usr/bin/env python

import pdb
import numpy as np
import math
from test_get_jacobian import get_jacobian
from test_fk_ik import get_fk
from test_control import *
import matplotlib.pyplot as plt

class DesiredTraj:
    def __init__(self, timespan, delta_t, r=None, num=None, shift=None):
        self.x, self.y, self.z, self.theta, self.phi, self.psi = np.zeros(timespan), np.zeros(timespan), np.zeros(
            timespan), np.zeros(timespan), np.zeros(timespan), np.zeros(timespan)
        self.xt, self.yt, self.zt, self.thetat, self.phit, self.psit = 0.125, 0.125, 0.568, 0, 0, 0
        self.r = r
        self.num = num
        self.shift = shift
        self.timespan = timespan
        self.delta_t = delta_t

    def get_instant_pose_circle(self):
        vy = 0
        vz = 0.5
        vth = 7
        delta_y = (vy * math.cos(self.thetat) - vz * math.sin(self.thetat)) * self.delta_t
        delta_z = (vy * math.sin(self.thetat) + vz * math.cos(self.thetat)) * self.delta_t
        delta_th = vth * self.delta_t
        self.yt += delta_y
        self.zt += delta_z
        self.thetat += delta_th
        traj_t = np.array([self.xt, self.yt, self.zt, 0, 0, 0])
        trajd_t = np.array([0, 0, 0, 0, 0, 0])
        return traj_t, trajd_t


class KineControl:
    def __init__(self, desired_traj):
        self.times = int(desired_traj.timespan/desired_traj.delta_t)
        self.num_joints = 7
        self.dsr_traj = desired_traj
        self.x_cur = [0, 0, 0, 0, 0, 0]
        self.q0 = np.array([0, math.pi / 3, 0, math.pi / 6, 0, 0, 0])
        self.q_dot0 = [0, 0, 0, 0, 0, 0, 0]
        self.error = 0
        self.error_lst = np.zeros(self.times)
        self.x_lst = np.zeros([self.times,6])
        self.qdot_lst = np.zeros([self.times,7])
        self.q_lst = np.zeros([self.times,7])
        self.x_dsr_lst = np.zeros([self.times,6])
        self.w_lst = np.zeros(self.times)
    def fk(self, q):
        T = np.zeros([8,4,4],dtype = float)
        DH_a = [0,0,0,0.0825,-0.0825,0,0.088,0]
        DH_d = [0.333,0,0.316,0,0.384,0,0,0.107]
        DH_alpha = [0,-0.5 * math.pi, 0.5 * math.pi, 0.5 * math.pi,-0.5 * math.pi,0.5 * math.pi,0.5 * math.pi,0]
        DH_theta =[q[0],q[1],q[2],q[3],q[4],q[5],q[6],0]
        for i in range(0,8):
            Tem = ([[math.cos(DH_theta[i]), -math.sin(DH_theta[i]), 0, DH_a[i]],
                    [math.sin(DH_theta[i]) * math.cos(DH_alpha[i]), math.cos(DH_theta[i]) * math.cos(DH_alpha[i]), -math.sin(DH_alpha[i]), -DH_d[i] * math.sin(DH_alpha[i])],
                    [math.sin(DH_theta[i]) * math.sin(DH_alpha[i]), math.cos(DH_theta[i]) * math.sin(DH_alpha[i]), math.cos(DH_alpha[i]), DH_d[i] * math.cos(DH_alpha[i])],
                    [0, 0, 0, 1]])
            if i == 0:
                Tem[0][3] = Tem[0][3]-0.42
                T[i] = Tem
            else:
                T[i] = np.matmul(T[i-1],Tem)
        pose_value = T
        return pose_value

    def jacobian(self, q):
        T = self.fk(q)
        z = np.zeros([7, 3], dtype = float)
        p = np.zeros([7, 3], dtype = float)
        z[0] = [0,0,1]
        p[0] = T[7,0:3,3]
        jacobian_matrix = np.zeros([6,7],dtype = float)
        for i in range(1,7):
            z[i,0] = T[i,0,2]
            z[i,1] = T[i,1,2]
            z[i,2] = T[i,2,2]
            p[i] = T[7,0:3,3]-T[i,0:3,3]
        for i in range(0,7):
            jacobian_matrix[0:3,i] = np.cross(z[i],p[i])
            jacobian_matrix[3, i] = z[i, 0]
            jacobian_matrix[4, i] = z[i, 1]
            jacobian_matrix[5, i] = z[i, 2]
        return jacobian_matrix

    def control(self):
        """
        Need to be implemented
        """

        dt = self.dsr_traj.delta_t
        ts = self.dsr_traj.timespan
        i = 0
        for t in np.arange(0, ts, dt).reshape(-1):
            print('=' * 80)
            x_dsr, xd_dsr = self.dsr_traj.get_instant_pose_circle()
            self.x_cur[0:2] = self.fk(self.q0)[7,0:2,3]
            self.x_lst[i] = self.x_cur
            self.q_lst[i] = self.q0
            self.qdot_lst[i] = self.q_dot0
            jacobian = self.jacobian(self.q0)
            jacobian_pseudoinverse = np.matmul(jacobian.transpose(),np.linalg.inv(np.matmul(jacobian, jacobian.transpose())))
            self.x_dsr_lst[i] = x_dsr
            dx = x_dsr - self.x_cur
            self.q_dot0 = np.matmul(jacobian_pseudoinverse, dx)
            self.w = math.sqrt(np.linalg.det(np.matmul(jacobian, jacobian.transpose())))
            self.w_lst[i] = self.w
            self.error_lst[i] = np.linalg.norm(dx)
            self.q0 += self.q_dot0
            self.x_cur += dx
            i = i+1

        # convert the whole trajectory with a shape of [ts/dt, 7]
        # into a 1-dim array [ts/dt * 7, ] by using '.reshape((-1,))'
        joint_goal_lst = self.q_lst.reshape((-1,))
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
        plt.figure(figsize = (12,4))
        plt.subplot(221)
        plt.plot(self.error_lst)
        plt.ylabel('tracking_error')
        plt.xlabel('time')
        plt.subplot(222)
        plt.plot(self.q_lst)
        plt.ylabel('joint_angles')
        plt.xlabel('time')
        plt.legend(['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'Joint7'])
        plt.subplot(223)
        plt.plot(self.qdot_lst)
        plt.ylabel('joint_velocities')
        plt.xlabel('time')
        plt.legend(['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'Joint7'])
        plt.subplot(224)
        plt.plot(self.w_lst)
        plt.ylabel('manipulability')
        plt.xlabel('time')
        plt.show()


if __name__ == '__main__':
    # You can define any trajectory you like, there is no limit.
    # If you define a new trajectory, the visualization script 'test_dsr_traj_vis.py' need to modify correspondingly.
    DsrTraj = DesiredTraj(timespan=1, delta_t=0.005, r=1, num=10, shift=10)
    kinecontroller = KineControl(DsrTraj)
    kinecontroller.control()
    kinecontroller.traj_vis()
