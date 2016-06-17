#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

fresh_data = false

def callback(data):
	global pose_target
	pose_target = data
    global fresh_data
    fresh_data = true
	
def kinect_planner():
	
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('kinect_trajectory_planner', anonymous=True,)
	rate = rospy.Rate(10)
	# Instantiate a RobotCommander object. 
	robot = moveit_commander.RobotCommander()
	
	# Instantiate a PlanningSceneInterface object (interface to the world surrounding the robot).
	scene = moveit_commander.PlanningSceneInterface()
	
	# Instantiate MoveGroupCommander objects for arms and Kinect2. 
	group = moveit_commander.MoveGroupCommander("Kinect2_Target")
	group_left_arm = moveit_commander.MoveGroupCommander("left_arm")
	group_right_arm = moveit_commander.MoveGroupCommander("right_arm")
	group_kinect = moveit_commander.MoveGroupCommander("neck")
	
	# We create this RobotTrajectory publisher to publish the computed trajectories.
	trajectory_publisher = rospy.Publisher("planned_path",
										RobotTrajectory, queue_size=5)

	## Getting info ot the reference frame for this robot
	group.set_pose_reference_frame('base_footprint')
	print "============ Reference frame: %s" % group.get_pose_reference_frame()
	print "============ End Effector: %s" % group.get_end_effector_link()
	
	## Set the planner for Moveit 
	group.set_planner_id("RRTConnectkConfigDefault")

	## Setting tolerance 
	group.set_goal_tolerance(0.03)
#    group.set_num_planning_attempts(10)

	print "=============== Current Pose ==========="
	print group.get_current_pose();

	# Suscribing to the desired pose topic
	target = rospy.Subscriber("desired_pose", PoseStamped, callback)
	
	# Define variable to store the calculated trajectory
	#plan_target = moveit_msgs.msg.RobotTrajectory()

	# Locating the arms and the Kinect2 sensor
	group_left_arm_values = group_left_arm.get_current_joint_values()
	group_right_arm_values = group_right_arm.get_current_joint_values()
	group_kinect_values = group_kinect.get_current_joint_values()
	
	
	## Planning to a Pose goal
	print "============ Generating plan"
	
	while not rospy.is_shutdown():
		if fresh_data: #If a new pose ir received, plan.
            
            # Update arms position
            group_left_arm_values = group_left_arm.get_current_joint_values()
            group_right_arm_values = group_right_arm.get_current_joint_values()
            
            # Set the target pose and generate a plan
            #group.set_pose_target(pose_target)
            plan_target = group.plan()
            group.go()
            
            trajectory_publisher.publish(plan_target)
            #trajectory_publisher.publish(group.plan())
            
            fresh_data = false
        
        rate.sleep()




if __name__=='__main__':
  try:
    move_group_interface()
  except rospy.ROSInterruptException:
    pass