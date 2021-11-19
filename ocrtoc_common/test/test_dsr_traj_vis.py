#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import pdb

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
import tf
import math

# 起始运动状态
x, y, th = 0, 0, 0


def DataUpdating(path_pub, path_record):
    """
    数据更新函数
    """
    global x, y, th

    # 时间戳
    current_time = rospy.Time.now()



    # 配置运动
    dt = 0.002
    vx = -1
    vy = 0
    vth = 3
    delta_x = (vx * math.cos(th) - vy * math.sin(th)) * dt
    delta_y = (vx * math.sin(th) + vy * math.cos(th)) * dt
    delta_th = vth * dt
    # pdb.set_trace()
    x += delta_x
    y += delta_y
    th += delta_th

    # 四元素转换
    quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # 配置姿态
    pose = PoseStamped()
    pose.header.stamp = current_time
    pose.header.frame_id = "world"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.5
    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]

    # 配置路径
    path_record.header.stamp = current_time
    path_record.header.frame_id = "world"
    path_record.poses.append(pose)

    # 路径数量限制
    if len(path_record.poses) > 1000:
        path_record.poses.pop(0)

    # 发布路径
    path_pub.publish(path_record)


def node():
    """
    节点启动函数
    """
    try:

        # 初始化节点path
        rospy.init_node('PathRecord', anonymous=True )

        # 定义发布器 path_pub 发布 trajectory
        path_pub = rospy.Publisher('dsr_traj', Path, queue_size=50)

        # 初始化循环频率
        rate = rospy.Rate(50)

        # 定义路径记录
        path_record = Path()

        # 发布tf
        # br = tf.TransformBroadcaster()
        # br.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
        #                  rospy.Time.now(), "odom", "world")

        # 在程序没退出的情况下
        while not rospy.is_shutdown():
            # 数据更新函数
            DataUpdating(path_pub, path_record)

            # 休眠
            rate.sleep()

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    node()
