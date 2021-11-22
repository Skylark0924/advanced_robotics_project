#!/usr/bin/env python
import pdb

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
import tf
import math

x, y, z, th = 0.125, 0.125, 0.5, 0


def DataUpdating(path_pub, path_record):
    global x, y, z, th

    current_time = rospy.Time.now()

    dt = 0.005
    vx = 0.5
    vy = 0
    vz = 0.5
    vth = 7
    # delta_x = (vx * math.cos(th) - vy * math.sin(th)) * dt
    # delta_y = (vx * math.sin(th) + vy * math.cos(th)) * dt
    delta_y = (vy * math.cos(th) - vz * math.sin(th)) * dt
    delta_z = (vy * math.sin(th) + vz * math.cos(th)) * dt
    delta_th = vth * dt
    # pdb.set_trace()
    # x += delta_x
    y += delta_y
    z += delta_z
    th += delta_th

    quat = tf.transformations.quaternion_from_euler(0, 0, 0)

    pose = PoseStamped()
    pose.header.stamp = current_time
    pose.header.frame_id = "world"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]

    path_record.header.stamp = current_time
    path_record.header.frame_id = "world"
    path_record.poses.append(pose)

    if len(path_record.poses) > 1000:
        path_record.poses.pop(0)

    path_pub.publish(path_record)


def node():
    try:
        rospy.init_node('PathRecord')

        path_pub = rospy.Publisher('dsr_traj', Path, queue_size=50)

        rate = rospy.Rate(50)

        path_record = Path()

        # br = tf.TransformBroadcaster()
        # br.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
        #                  rospy.Time.now(), "odom", "world")

        while not rospy.is_shutdown():
            DataUpdating(path_pub, path_record)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    node()
