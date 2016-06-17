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
    global trajectory1
    global fresh_data
    
    trajectory1 = data
    fresh_data = true

def kinect_controller():

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('robot_state_publisher', anonymous=True,)
    
    rate = rospy.Rate(10)
    
    # Instantiate a RobotCommander object. 
    robotc = moveit_commander.RobotCommander()
    groupc = moveit_commander.MoveGroupCommander("Kinect2_Target")
    groupc.set_planner_id("RRTConnectkConfigDefault")

    # We create this publisher for the desired pose of the Kinect2 Sensor
    pose_publisher = rospy.Publisher("desired_pose", PoseStamped, queue_size=5)
    
    # Create a suscrber to read the trajectory published by kinect_planner.py
    rospy.Subscriber("planned_path", RobotTrajectory, callback)

    print "=============== Current Pose ==========="
    print groupc.get_current_pose();
    
    print "========= Setting initial pose ================"
    group_variable_values = groupc.get_current_joint_values()
    
    ## Defining a Pose goal
    pose_target = PoseStamped()
    pose_target.header.frame_id = 'base_footprint'
    pose_target.pose.orientation.x = 0 
    pose_target.pose.orientation.y = 0 
    pose_target.pose.orientation.z = 0 
    pose_target.pose.orientation.w = 1 
    pose_target.pose.position.x =  1.3 
    pose_target.pose.position.y = -0.2 
    pose_target.pose.position.z = 1.17
    

    while not rospy.is_shutdown():
        global fresh_data
        
        # Publishe desired pose
        pose_publisher.publish(pose_target)
        fresh_data = false
        
        # If no trajectory is received, sleep
        while fresh_data == false:
            rate.sleep()
        
        # Read the trajectory published by kinect_planner.py
        #plan = set_robot_trajectory_msg(trajectory1,group_variable_values)
        plan_target = groupc.plan(trajectory1)
        groupc.go(wait=True)
        print "============ Waiting while the plan is executed"
        rate.sleep()
        
        # Display current joint values
        group_variable_values = groupc.get_current_joint_values()
        print "============ Joint values: ", group_variable_values



if __name__=='__main__':
  try:
    move_group_interface()
  except rospy.ROSInterruptException:
    pass