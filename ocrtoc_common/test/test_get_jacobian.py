#! /usr/bin/env python

import rospy
from ocrtoc_common.srv import *
import numpy as np
import pdb


def get_jacobian(joint_value):
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
    rospy.wait_for_service('/get_jacobian')
    try:
        service_call = rospy.ServiceProxy('/get_jacobian', Jacobian)
        request = JacobianRequest()
        request.joint_value = joint_value
        response = service_call(request)
        jacobian_matrix = np.array(response.jacobian).reshape((6, 7))
        print('\033[1;33mJacobian matrix:\033[0m\n', jacobian_matrix)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
    return jacobian_matrix


if __name__ == '__main__':
    a = [0.88, -0.22, -0.19, -2.41, -0.645, 1.51, 0.08]
    get_jacobian(a)
