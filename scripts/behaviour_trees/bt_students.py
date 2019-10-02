#!/usr/bin/env python

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty, SetBool, SetBoolRequest  

class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):

		rospy.loginfo("Initialising behaviour tree")

		# tuck the arm
		b0 = activate_localizator()

		b1 = move_there()
		# go to table
		b2 = pt.composites.Selector(
			name="Go to table fallback",
			children=[counter(5, "At table?"), go("Go to table!", 0, -1)]
		)

		# become the tree
		tree = RSequence(name="Main sequence", children=[b0, b1])
		super(BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)	




#################################
self.mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')


####
# TOPIC to pub Rotation after starting localization service
self.cmd_vel_pub = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
move_msg = Twist()
move_msg.angular.z = 1
rate = rospy.Rate(10)
cnt = 0
rospy.loginfo("%s: Moving towards door", self.node_name)
while not rospy.is_shutdown() and cnt < 25:
	self.cmd_vel_pub.publish(move_msg)
	rate.sleep()
	cnt = cnt + 1
####




#####
# Set up ACTION clients
rospy.loginfo("%s: Waiting for play_motion action server...", self.node_name)
self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
if not self.play_motion_ac.wait_for_server(rospy.Duration(1000)):
	rospy.logerr("%s: Could not connect to /play_motion action server", self.node_name)
	exit()
rospy.loginfo("%s: Connected to play_motion action server", self.node_name)

# Call action service
rospy.loginfo("%s: Tucking the arm...", self.node_name)
goal = PlayMotionGoal()
goal.motion_name = 'home'
goal.skip_planning = True
self.play_motion_ac.send_goal(goal)
success_tucking = self.play_motion_ac.wait_for_result(rospy.Duration(100.0))

if success_tucking:
	rospy.loginfo("%s: Arm tucked.", self.node_name)
	self.state = 2
else:
	self.play_motion_ac.cancel_goal()
	rospy.logerr("%s: play_motion failed to tuck arm, reset simulation", self.node_name)
	self.state = 5
#####


#### SERVICE global_localization
class activate_localizator(pt.behaviour.Behaviour):

	def __init__(self):

		# server
		rospy.wait_for_service('/global_localization', timeout=30)
		self.move_head_srv = rospy.ServiceProxy('/global_localization', Empty)

		# execution checker
		self.activated = False

		# become a behaviour
		super(activate_localizator, self).__init__("Activate localizator!")

	def update(self):

		# try to tuck head if haven't already
		if not self.activated:

			# command
			self.move_head_srv()
			self.activated = True

			# tell the tree you're running
			return pt.common.Status.SUCCESS

		# react to outcome
		else: return pt.common.Status.SUCCESS
####

#### ACTION SERVICE move_base
class TuckArm(pt.behaviour.Behaviour):

	def __init__(self):

		# Set up action client
		self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

		# personal goal setting
		self.goal = PlayMotionGoal()
		self.goal.motion_name = 'home'
		self.goal.skip_planning = True

		# execution checker
		self.sent_goal = False
		self.finished = False

		# become a behaviour
		super(TuckArm, self).__init__("Tuck arm!")

	def update(self):

		# already tucked the arm
		if self.finished: 
			return pt.common.Status.SUCCESS
		
		# command to tuck arm if haven't already
		elif not self.sent_goal:

			# send the goal
			self.play_motion_ac.send_goal(self.goal)
			self.sent_goal = True

			# tell the tree you're running
			return pt.common.Status.RUNNING

		# if I was succesful! :)))))))))
		elif self.play_motion_ac.get_result():

			# than I'm finished!
			self.finished = True
			return pt.common.Status.SUCCESS

		# if I'm still trying :|
		else:
			return pt.common.Status.RUNNING
		

################################


if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()