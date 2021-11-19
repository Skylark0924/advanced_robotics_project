#! /usr/bin/env python

# import rospy
import pdb
import numpy as np
import math
from test_control import *
from test_get_jacobian import *
from test_fk_ik import *


# import tf


# from ocrtoc_common.srv import *


class DesiredTraj:
    def __init__(self, x0, y0, z0, mode, timespan, delta_t, r=None, num=None, shift=None):
        self.x, self.y, self.z, self.theta, self.phi, self.psi = np.zeros(timespan), np.zeros(timespan), np.zeros(
            timespan), np.zeros(timespan), np.zeros(timespan), np.zeros(timespan)
        self.x0, self.y0, self.z0, self.theta0, self.phi0, self.psi0 = x0, y0, z0, 0, 0, 0
        self.xt, self.yt, self.zt, self.thetat, self.phit, self.psit = 0.125, 0.125, 0.5, 0, 0, 0

        self.mode = mode
        self.r = r
        self.num = num
        self.shift = shift
        self.timespan = timespan
        self.delta_t = delta_t

    def get_instant_pose_circle(self, t):
        # x = self.x0 + self.r * math.cos(t * 2 * math.pi)
        # y = self.y0
        # z = self.z0 + self.r * math.sin(t * 2 * math.pi)
        # theta = 0.1 * math.sin(t * 2 * math.pi)
        # phi = self.phi0
        # psi = self.psi0

        dt = 0.005
        vx = 0.5
        vy = 0
        vz = 0.5
        vth = 7
        # delta_x = (vx * math.cos(th) - vy * math.sin(th)) * dt
        # delta_y = (vx * math.sin(th) + vy * math.cos(th)) * dt
        delta_y = (vy * math.cos(self.thetat) - vz * math.sin(self.thetat)) * dt
        delta_z = (vy * math.sin(self.thetat) + vz * math.cos(self.thetat)) * dt
        delta_th = vth * dt
        # pdb.set_trace()
        # x += delta_x
        self.y += delta_y
        self.z += delta_z
        self.thetat += delta_th
        # self.xt += delta_x
        # self.yt += delta_y
        # self.zt = 0.6
        # self.thetat += delta_th

        # quat = tf.transformations.quaternion_from_euler(0, 0, th)

        traj_t = np.array([self.xt, self.yt, self.zt, self.thetat, self.phit, self.psit])
        trajd_t = np.array([0, 0, 0, 0, 0, 0])

        return traj_t, trajd_t

    def get_instant_pose_helix(self, t):
        x = self.x0 + self.r * math.cos(t * self.num * 2 * math.pi)
        y = self.y0 + t * self.num * self.shift
        z = self.z0 + self.r * math.sin(t * self.num * 2 * math.pi)
        theta = self.theta0
        phi = self.phi0
        psi = self.psi0

        traj_t = np.array([x, y, z, theta, phi, psi])
        trajd_t = np.array([0, 0, 0, 0, 0, 0])

        return traj_t, trajd_t

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
        # self.Q = np.zeros((self.num_joints, self.dsr_traj))
        # self.dQ = np.zeros((self.num_joints, self.dsr_traj))
        # self.ddQ = np.zeros((self.num_joints, self.dsr_traj))
        # self.TAU = np.zeros((self.num_joints, self.dsr_traj))

        self.q0 = np.array([0.88, -0.22, -0.19, -1.41, -0.645, 1.51, 0.08])
        self.q_dot0 = [0, 0, 0, 0, 0, 0, 0]
        self.pos0 = self.fk(self.q0)

    def fk(self, q):
        pose_value = get_fk(q)
        return pose_value

    def jacobian(self, q):
        jacobian_matrix = get_jacobian(q)
        return jacobian_matrix

    def quart_to_rpy(self, p_quart):
        position = p_quart[:3]
        x, y, z, w = p_quart[3:]

        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(2 * (w * y - x * z))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))

        p_euler = np.hstack((position, np.array([roll, pitch, yaw])))
        return p_euler

    def control(self):
        K = 100 * np.diag(np.ones((6,)))
        i = 1
        dt = self.dsr_traj.delta_t
        ts = self.dsr_traj.timespan

        # q_r --> record the joint degrees
        # qd_r --> record the joint velocity
        q_r = np.zeros([1, self.num_joints])
        qd_r = np.zeros([1, self.num_joints])
        W = np.zeros([1, 1])
        # e_r --> record the endtip position error
        e_r = np.zeros([1, 1])

        # p_r --> record the real end-effector positions
        # x1_r = np.zeros([self.num_joints, 1])
        p_r = np.zeros([1, 6])

        # the initial state
        q = self.q0

        for t in np.arange(0, ts, dt).reshape(-1):
            print('=' * 80)

            x_dsr, xd_dsr = self.dsr_traj.get_instant_pose_circle(t)
            print('\033[1;33mDesired pose value:\033[0m\n', x_dsr)

            # current state
            q_r = np.append(q_r, q.reshape((1, -1)), axis=0)
            p = self.fk(q.reshape((-1,)))
            p = self.quart_to_rpy(p)
            # p_ori = tf.transformations.euler_from_quaternion(p[3:])
            # p = np.hstack(p[:3], p_ori)

            p_r = np.append(p_r, p.reshape((1, -1)), axis=0)

            # control law
            e = x_dsr - p
            J = self.jacobian(q.reshape((-1,)))

            # pdb.set_trace()

            # manupilablity
            mani = np.sqrt(np.linalg.det(J @ J.T))
            # if mani < 0.01:
            #     pdb.set_trace()
            W = np.append(W, np.array(mani).reshape(1, -1), axis=0)

            J_pseu = J.T @ np.linalg.inv(J @ J.T)
            # J_pseu = np.linalg.inv(J)

            # pdb.set_trace()

            qd_r = np.append(qd_r, np.array(J_pseu @ (xd_dsr + K @ e)).reshape((1, -1)), axis=0)

            print('\033[1;33mJoint value:\033[0m\n', q)

            q = q.reshape((1, -1)) + qd_r[i, :].reshape((1, -1)) * dt

            # error
            e_r = np.append(e_r, np.sqrt(e.T @ e))
            i = i + 1
        print()
        joint_goal_lst = q_r.reshape((-1,))
        execute_joint_traj_goal(joint_goal_lst)
        pdb.set_trace()

    def traj_vis(self):
        ...


if __name__ == '__main__':
    DsrTraj = DesiredTraj(x0=0, y0=0, z0=0, mode='circle', timespan=10, delta_t=0.005, r=1, num=10, shift=10)
    # desired_traj = Traj.get_desired_pose()

    kinecontroller = KineControl(DsrTraj)
    kinecontroller.control()
    kinecontroller.traj_vis()
