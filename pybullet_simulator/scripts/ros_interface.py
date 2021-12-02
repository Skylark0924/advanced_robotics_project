#!/usr/bin/env python
# license removed for brevity
import pdb

import pybullet, os
import os.path as path
import numpy as np
from transforms3d.euler import quat2euler
import time

import actionlib
import control_msgs.msg
import rospkg
import rospy
from std_msgs.msg import String
import trajectory_msgs.msg
import tf
import tf.transformations

from pybullet_env import Box, SimRobot, Manipulator, Camera

ROBOT_ID_1 = 1
ROBOT_ID_2 = 2
# BOX_ID = 2
PYBULLET_JOINT_POS_ID = 14
PYBULLET_JOINT_ORI_ID = 15


class ArmControlAction(object):
    _result = control_msgs.msg.FollowJointTrajectoryResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                control_msgs.msg.FollowJointTrajectoryAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True

        # publish info to the console for the user
        rospy.loginfo('%s: Executing' % (self._action_name))
        robot.executeArmJointTrajectory(goal.trajectory)

        if success:
            # self._result.position = self._feedback.position
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._result.error_code = 0
            self._as.set_succeeded(self._result)


class GripperControlAction(object):
    # create messages that are used to publish feedback/result
    _feedback = control_msgs.msg.GripperCommandFeedback()
    _result = control_msgs.msg.GripperCommandResult()
    _goal = control_msgs.msg.GripperCommandGoal()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.GripperCommandAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True

        # publish info to the console for the user
        rospy.loginfo('%s: Executing, goal %f' % (self._action_name, goal.command.position))
        robot.gripperControl(gripper_opening_length=goal.command.position * 2, T=1.0)

        if success:
            self._result.position = self._feedback.position
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


def kinect_callback(event):
    kinect.updateCameraImage(kinect.getBasePosition(), kinect.getBaseOrientation())
    kinect.ros_publish_image()


if __name__ == '__main__':
    rospy.init_node('ros_interface', anonymous=True)

    # connect to simulation
    physics_id = pybullet.connect(pybullet.SHARED_MEMORY)
    if physics_id == -1:
        print('pybullet not connected!')
    else:
        print("connected to pybullet physics id:", physics_id)

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('pybullet_simulator')
    # get robot
    panda_config_path_1 = pkg_path + '/robots/franka/config/panda_arm_hand_1.yaml'
    panda_config_path_2 = pkg_path + '/robots/franka/config/panda_arm_hand_2.yaml'

    # Wait until robot is loaded in pybullet
    robot_loaded_flag = False
    while not robot_loaded_flag:
        pybullet.disconnect()
        pybullet.connect(pybullet.SHARED_MEMORY)
        num_robot_joint = pybullet.getNumJoints(ROBOT_ID_1)
        rospy.loginfo('\033[1;33mnumber of joints of\033[0m {}:{}'.format(ROBOT_ID_1, num_robot_joint))
        if num_robot_joint == 0:
            time.sleep(1)
            continue
        # robot_1 = Manipulator(verbose=True, urdf_path=None, config_path=panda_config_path_1, from_id=True, id=1)
        # robot_2 = Manipulator(verbose=True, urdf_path=None, config_path=panda_config_path_2, from_id=True, id=2)
        robot_1 = Manipulator(verbose=False, config_path=panda_config_path_1, from_id=True, id=1)
        robot_2 = Manipulator(verbose=False, config_path=panda_config_path_2, from_id=True, id=2)
        # robot_1.loadFromID(id=1, config_path=panda_config_path_1)
        # robot_2.loadFromID(id=2, config_path=panda_config_path_2)
        # robot_1 = Manipulator.loadFromID()
        # robot_2 = Manipulator.loadFromID(id=2, config_path=panda_config_path_1)
        robot_loaded_flag = True
    print('\033[1;33mROS robot ID:\033[0m {}, {}'.format(robot_1.id, robot_2.id))
    rospy.loginfo('Robot loaded')
    # pdb.set_trace()

    # get table
    # table = Box.fromID(id=BOX_ID)

    # # kinect
    # ##### ! The transformation is hardcoded here and it should be consistent with that in panda_arm_hand_realsense.urdf #####
    # kinect_urdf_path = pkg_path + '/robots/kinect/kinect.urdf'
    # kinect = Camera(name='kinect')
    # num_robot_joint = pybullet.getNumJoints(ROBOT_ID)
    # kinect_id = None
    # for link_id in range(num_robot_joint):
    #     joint_info = pybullet.getJointInfo(ROBOT_ID, link_id)
    #     if joint_info[1] == 'kinect_camera_joint':
    #         kinect_id = link_id
    #         break
    # if kinect_id is None:
    #     raise ValueError('Cannot find kinect joint in URDF')
    # else:
    #     kinect_joint_info = pybullet.getJointInfo(ROBOT_ID, kinect_id)
    #     kinect_base_position = robot.getBasePosition() + np.array(kinect_joint_info[PYBULLET_JOINT_POS_ID])
    #     print('kinect base position:{}'.format(kinect_base_position))
    #     qx, qy, qz, qw = np.array(kinect_joint_info[PYBULLET_JOINT_ORI_ID])
    #     kinect_base_rpy = quat2euler([qw, qx, qy, qz])
    #     print('kinect base rpy:{}'.format(kinect_base_rpy))
    #     kinect.loadURDF(kinect_urdf_path, basePosition=kinect_base_position, baseRPY=kinect_base_rpy, useFixedBase=True)
    #
    # # realsense
    # realsense = Camera(name='realsense')

    # tf
    tf_broadcaster = tf.TransformBroadcaster()

    # joint state
    from sensor_msgs.msg import JointState, CameraInfo, Image

    joint_state_publisher_1 = rospy.Publisher('/joint_state_controller/joint_states_1', JointState, queue_size=10)
    joint_state_publisher_1_1 = rospy.Publisher('/joint_states_1', JointState, queue_size=10)
    joint_state_publisher_2 = rospy.Publisher('/joint_state_controller/joint_states_2', JointState, queue_size=10)
    joint_state_publisher_2_1 = rospy.Publisher('/joint_states_2', JointState, queue_size=10)

    # kinect.init_ros_publiser()
    # realsense.init_ros_publiser()

    arm_control_action_server = ArmControlAction('/position_joint_trajectory_controller/follow_joint_trajectory')
    gripper_control_action_server = GripperControlAction('/franka_gripper/gripper_action')

    rate = rospy.Rate(5)
    robot_1.showRobotInfo()
    robot_2.showRobotInfo()

    # rospy.Timer(rospy.Duration(1), kinect_callback, oneshot=False)

    while not rospy.is_shutdown():
        # tf
        robot_1.broadcast_tfs()
        robot_2.broadcast_tfs()

        # joint state
        joint_info = robot_1.getArmJointStateMsg()
        joint_state_publisher_1.publish(joint_info)
        joint_state_publisher_1_1.publish(joint_info)

        joint_info_2 = robot_2.getArmJointStateMsg()
        joint_state_publisher_2.publish(joint_info_2)
        joint_state_publisher_2_1.publish(joint_info_2)

        # camera
        # realsense.updateCameraImage(robot.getRealsensePosition(), robot.getRealsenseOrientation())
        # realsense.ros_publish_image()

        rate.sleep()
