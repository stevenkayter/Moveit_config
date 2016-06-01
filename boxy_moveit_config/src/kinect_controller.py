#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String

def kinect_controller():

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('robot_state_publisher',
                    anonymous=True,)

    # Instantiate a RobotCommander object. 
    robot = moveit_commander.RobotCommander()

    # Instantiate a MoveGroupCommander object. 
    group = moveit_commander.MoveGroupCommander("Kinect2_Target")

    # We create this publisher to publish the desired pose of the Kinect2 Sensor
    display_trajectory_publisher = rospy.Publisher(
                                        '/move_group/desired_pose',
#                                        moveit_msgs.msg.MotionPlanRequest, queue_size=10)
                                         geometry_msgs.msg.PoseStamped, queue_size=10)

    print "============ Waiting for RVIZ..."
    rospy.sleep(8)

    print "=============== Current Pose ==========="
    print group.get_current_pose();
    print "========================================="
    
    print "========= Setting initial pose ================"
    group_variable_values = group.get_current_joint_values()
    
    ## Defining a Pose goal
    print "============ Defining Pose 1"

    pose_target = geometry_msgs.msg.PoseStamped()
    pose_target.header.frame_id = 'base_footprint'
    pose_target.pose.orientation.x = 0 
    pose_target.pose.orientation.y = 0 
    pose_target.pose.orientation.z = 0 
    pose_target.pose.orientation.w = 1 
    pose_target.pose.position.x =  1.3 
    pose_target.pose.position.y = -0.2 
    pose_target.pose.position.z = 1.17
    
    # Read the trajectory published by kinect_planner.py
    trajectory1 = moveit_msgs.msg.RobotTrajectory

    while not rospy.is_shutdown():

        group.go(wait=True)
        print "============ Waiting while the plan is executed"
        rospy.sleep(3)
        
        # Display current joint values
        group_variable_values = group.get_current_joint_values()
        print "============ Joint values: ", group_variable_values


    ## Adding/Removing Objects and Attaching/Detaching Objects
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will define the collision object message
    collision_object = moveit_msgs.msg.CollisionObject()

    ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()


    print "============ STOPPING"


if __name__=='__main__':
  try:
    move_group_interface()
  except rospy.ROSInterruptException:
    pass