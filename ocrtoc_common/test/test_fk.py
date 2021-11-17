#!/usr/bin/env python

import rospy
import moveit_commander

from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionFK, GetPositionFKRequest
from moveit_msgs.msg import RobotState, DisplayRobotState, MoveItErrorCodes

class ReachabilityTest:
	def __init__(self):
		self.ik= rospy.ServiceProxy("compute_ik", GetPositionIK)
		self.fk= rospy.ServiceProxy("compute_fk", GetPositionFK)

		self.robot= moveit_commander.RobotCommander()
		self.group1= moveit_commander.MoveGroupCommander(rospy.get_param("~group1", "arm"))
		self.group2= moveit_commander.MoveGroupCommander(rospy.get_param("~group2", self.group1.get_name()))

		self.pub_state= rospy.Publisher("ik_benchmarks/test_state", DisplayRobotState, queue_size=1)
		self.ik.wait_for_service()
		self.fk.wait_for_service()

		rospy.sleep( rospy.Duration(0.2) )

	def test(self, joint_values):
		print(str(joint_values)+" ")
		fkr= GetPositionFKRequest()
		fkr.header.frame_id= self.group1.get_planning_frame()
		fkr.fk_link_names= [self.group1.get_end_effector_link()]
		fkr.robot_state= self.robot.get_current_state()
		fkr.robot_state.joint_state.position= list(fkr.robot_state.joint_state.position)

		for (j,v) in zip(self.group1.get_active_joints(), joint_values):
			fkr.robot_state.joint_state.position[fkr.robot_state.joint_state.name.index(j)]= v

		target= self.fk(fkr).pose_stamped[0]

		self.pub_state.publish( DisplayRobotState(state= fkr.robot_state) )

		ik_seed= self.robot.get_current_state()
		ik_seed.joint_state.position= [0.0] * len(ik_seed.joint_state.position)
		ikr= GetPositionIKRequest()
		ikr.ik_request.group_name= self.group1.get_name()
		ikr.ik_request.robot_state= ik_seed
		ikr.ik_request.ik_link_name= self.group1.get_end_effector_link()
		ikr.ik_request.pose_stamped= target
		ikr.ik_request.timeout= rospy.Duration(rospy.get_param("~timeout", 6.0))
		response= self.ik(ikr)
		if response.error_code.val == MoveItErrorCodes.SUCCESS:
			print "OK "
		else:
			print "FAILED "

#		ikr.ik_request.group_name= self.group2.get_name()
#		response= self.ik(ikr)
#		if response.error_code.val == MoveItErrorCodes.SUCCESS:
#			print "OK "
#		else:
#			print "FAILED "
#			raise Exception("ik failed")

	def run(self):
		if self.group1.get_active_joints() != self.group2.get_active_joints():
			rospy.logfatal(self.group1.get_name() + " and " + self.group2.get_name() + " do not represent the same group")

		rospy.loginfo("Running IK reachability tests between " + self.group1.get_name() + " and " + self.group2.get_name())
		while not rospy.is_shutdown():
			self.test(self.group1.get_random_joint_values())


if __name__ == '__main__':
	rospy.init_node("ik_reachability_test")
	test= ReachabilityTest()
	test.run()