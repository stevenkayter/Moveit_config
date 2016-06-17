#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

fresh_data = False
pose_target = PoseStamped()

def callback(data):
    global pose_target
    global fresh_data
    pose_target = data   
    print "------------- Target pose: ", data
    fresh_data = True

def kinect_planner():
    global fresh_data
    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('kinect_trajectory_planner', anonymous=True)
    rate = rospy.Rate(2)
    print "===================== Here 1 ======================="
    rospy.sleep(3)
       
    # Instantiate a RobotCommander object. 
    robot = moveit_commander.RobotCommander()
    
    # Instantiate a PlanningSceneInterface object (interface to the world surrounding the robot).
    scene = moveit_commander.PlanningSceneInterface()
    print "===================== Here 2 ======================="
    
    # Instantiate MoveGroupCommander objects for arms and Kinect2. 
    group = moveit_commander.MoveGroupCommander("Kinect2_Target")
    group_left_arm = moveit_commander.MoveGroupCommander("left_arm")
    group_right_arm = moveit_commander.MoveGroupCommander("right_arm")
    group_kinect = moveit_commander.MoveGroupCommander("neck")
    print "===================== Here 3 ======================="
    
    
    # We create this DisplayTrajectory publisher to publish
    # trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                        moveit_msgs.msg.DisplayTrajectory, queue_size=5)

    
    ## Set the planner for Moveit 
    group.set_planner_id("RRTConnectkConfigDefault")
    group.set_planning_time(5)
    group.set_pose_reference_frame('base_footprint')
    
    ## Setting tolerance 
    group.set_goal_tolerance(0.03)
    #    group.set_num_planning_attempts(10)
    
    print "=============== Current Pose ==========="
    print group.get_current_pose();
    
    # Suscribing to the desired pose topic
    target = rospy.Subscriber("desired_pose", PoseStamped, callback)
    
    # Locating the arms and the Kinect2 sensor
    group_left_arm_values = group_left_arm.get_current_joint_values()
    group_right_arm_values = group_right_arm.get_current_joint_values()
    group_kinect_values = group_kinect.get_current_joint_values()
    
    while not rospy.is_shutdown():
        if fresh_data == True: #If a new pose ir received, plan.
        
            # Update arms position
            group_left_arm_values = group_left_arm.get_current_joint_values()
            group_right_arm_values = group_right_arm.get_current_joint_values()
            
            # Set the target pose and generate a plan
            group.set_pose_target(pose_target)
            plan_target = group.plan()
            group.go(wait=True)
            
            rate.sleep()
    
        fresh_data = False
    
        rate.sleep()


if __name__=='__main__':
  try:
    kinect_planner()
  except rospy.ROSInterruptException:
    pass