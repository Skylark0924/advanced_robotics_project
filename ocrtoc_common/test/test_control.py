#! /usr/bin/env python

import rospy
from ocrtoc_common.srv import *
import numpy as np
import pdb


def get_manipulator_info():
    """
    Example:
        group_names:
          - hand
          - panda_arm
          - panda_arm_hand
        end_effector_link: "panda_link8"
        planning_frame: "world"
    """
    print('=' * 80)
    rospy.wait_for_service('/get_manipulator_info')
    try:
        service_call = rospy.ServiceProxy('/get_manipulator_info', ManipulatorInfo)
        request = ManipulatorInfoRequest()
        response = service_call(request)
        print('\033[1;33mManipulator info:\033[0m\n\n', response)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def get_manipulator_state():
    """
    Example:
        current_pose:
          header:
            seq: 0
            stamp:
              secs: 1637122023
              nsecs: 372070074
            frame_id: "world"
          pose:
            position:
              x: -0.21322741402002
              y: 0.33390438460817584
              z: 0.3792184581268574
            orientation:
              x: 0.713732306560555
              y: 0.5557741554521883
              z: -0.08399854533215535
              w: 0.41790612233519464
        joint_position_list: [1.379531790886312, -0.22001797639057735, -0.19007410841535446, -2.4099934193045796, -0.6449946037573167, 1.5100918667436882, 0.07997906447997524]
    """
    # print('=' * 80)
    rospy.wait_for_service('/get_manipulator_state')
    try:
        service_call = rospy.ServiceProxy('/get_manipulator_state', ManipulatorState)
        request = ManipulatorStateRequest()
        response = service_call(request)
        print('\033[1;33mManipulator state:\033[0m\n', response)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
    return response


def execute_joint_goal(joint_goal):
    # rospy.sleep(2.0)
    request = JointSpaceGoalRequest()
    request.joint_goal = joint_goal
    # print('=' * 80)
    print(request.joint_goal)
    rospy.wait_for_service('/send_joint_space_goal')
    try:
        service_call = rospy.ServiceProxy('/send_joint_space_goal', JointSpaceGoal)
        response = service_call(request)
        print('\033[1;33mExecution info:\033[0m\n', response)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def execute_joint_traj_goal(joint_traj_goal):
    # rospy.sleep(2.0)
    start_joint_state = get_manipulator_state().joint_position_list
    joint_traj_goal = list(start_joint_state) + list(joint_traj_goal)
    # pdb.set_trace()

    request = JointTrajGoalRequest()
    request.joint_traj_goal = joint_traj_goal
    # print('=' * 80)
    print(request.joint_traj_goal)
    rospy.wait_for_service('/send_joint_traj_goal')
    try:
        service_call = rospy.ServiceProxy('/send_joint_traj_goal', JointTrajGoal)
        response = service_call(request)
        print('\033[1;33mExecution info:\033[0m\n', response)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def execute_ee_pose_goal(pose_goal):
    request = PoseGoalRequest()
    # Goal in 'world' frame
    request.goal.position.x = pose_goal[0]
    request.goal.position.y = pose_goal[1]
    request.goal.position.z = pose_goal[2]
    request.goal.orientation.x = pose_goal[3]
    request.goal.orientation.y = pose_goal[4]
    request.goal.orientation.z = pose_goal[5]
    request.goal.orientation.w = pose_goal[6]
    print(request.goal)
    rospy.wait_for_service('/send_pose_goal')
    try:
        service_call = rospy.ServiceProxy('/send_pose_goal', PoseGoal)
        response = service_call(request)
        print('\033[1;33mExecution info:\033[0m\n', response)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    get_manipulator_info()
    get_manipulator_state()

    # joint_goal_lst = []
    # for i in range(10):
    #     a = [0.48 + i / 10.0, -0.22, -0.19, -2.41, -0.645, 1.51, 0.08]
    #     joint_goal_lst.append(a)
    #
    # joint_goal_lst = np.array(joint_goal_lst).reshape((-1, )).tolist()
    #
    # execute_joint_traj_goal(joint_goal_lst)

    # pose_goal = [-0.343232254619, -0.525294298568, 0.488844847508, -0.332872961262, 0.623855146947, 0.334828632737,
    # 0.622808264226]

    a = [2.89729998, -1.04693991, -0.60678114, -1.37532981, -2.15888884, 0.55902705,
         0.2286122]
    b = [0.88, -0.22, -0.19, -1.41, -0.645, 1.51, 0.08]

    execute_joint_goal(b)
