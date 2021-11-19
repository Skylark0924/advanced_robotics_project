import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Empty


class ProjectElement(object):
    def __init__(self):
        self.path_pub = rospy.Publisher('~path', Path, latch=True, queue_size=10)
        self.circle_sub = rospy.Subscriber('~circle', Empty, self.circle_cb, queue_size=10)
        # self.line_sub = rospy.Subscriber('~line', Empty, self.line_cb, queue_size=10)
        self.project_sub = rospy.Subscriber('~project', Empty, self.project_cb, queue_size=10)

        self.paths = []

        self.rate = rospy.Rate(50)

    def circle_cb(self, msg):

        path = Path()

        centre_x = 1
        centre_y = 1
        R = 0.5
        th = 0.0
        delta_th = 0.1

        while (th < 2 * math.pi):
            x = centre_x + R * math.sin(th)
            y = centre_y + R * math.cos(th)
            th += delta_th

            this_pose_stamped = PoseStamped()
            this_pose_stamped.pose.position.x = x
            this_pose_stamped.pose.position.y = y

            this_pose_stamped.header.stamp = rospy.get_rostime()
            this_pose_stamped.header.frame_id = "world"

            path.poses.append(this_pose_stamped)

        path.header.frame_id = "world"
        path.header.stamp = rospy.get_rostime()

        self.paths.append(path)

    def line_cb(self, msg):

        path = Path()

        x_start = 0.0
        y_start = 0.0
        length = 2
        angle = 45 * math.pi / 180
        th = 0.0
        delta_th = 0.1

        while (th < length):
            x = x_start + th * math.cos(angle)
            y = y_start + th * math.sin(angle)
            th += delta_th

            this_pose_stamped = PoseStamped()
            this_pose_stamped.pose.position.x = x
            this_pose_stamped.pose.position.y = y

            this_pose_stamped.header.stamp = rospy.get_rostime()
            this_pose_stamped.header.frame_id = "world"

            path.poses.append(this_pose_stamped)

        path.header.frame_id = "world"
        path.header.stamp = rospy.get_rostime()

        self.paths.append(path)

    def project_cb(self, msg):

        while (True):
            for element in self.paths:
                # element.header.stamp = rospy.get_rostime()
                self.path_pub.publish(element)


if __name__ == '__main__':
    rospy.init_node('path_simulate')

    elements = ProjectElement()

    rospy.spin()