#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String

def move_group_interface():

  ## Setup
  ## ^^^^^
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('robot_state_publisher',
                    anonymous=True)

    # Instantiate a RobotCommander object. 
    robot = moveit_commander.RobotCommander()

    # Instantiate a PlanningSceneInterface object.
    # This object is an interface to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()

    # Instantiate a MoveGroupCommander object. 
    group = moveit_commander.MoveGroupCommander("Kinect2_Target")
    group_left_arm = moveit_commander.MoveGroupCommander("left_arm")
    group_right_arm = moveit_commander.MoveGroupCommander("right_arm")
    group_kinect = moveit_commander.MoveGroupCommander("neck")

    # We create this DisplayTrajectory publisher to publish
    # trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher(
                                        '/move_group/display_planned_path',
                                        moveit_msgs.msg.DisplayTrajectory, queue_size=10)

    print "============ Waiting for RVIZ..."
    rospy.sleep(8)

    ## We can get the name of the reference frame for this robot
    group.set_pose_reference_frame('base_footprint')
    print "============ Reference frame: %s" % group.get_pose_reference_frame()
    print "============ End Effector: %s" % group.get_end_effector_link()

    ## Set the planner for Moveit (yo)
    group.set_planner_id("RRTConnectkConfigDefault")
#    group.set_planner_id("ESTkConfigDefault")
#    group.set_planner_id("KPIECEkConfigDefault")
#    group.set_planner_id("PRMkConfigDefault")
#    group.set_planner_id("PRMstarkConfigDefault")
#    group.set_planner_id("RRTkConfigDefault")

    ## Setting tolerance (yo) --------
    group.set_goal_tolerance(0.03)
#    group.set_num_planning_attempts(10)

    print "=============== Current Pose ==========="
    print group.get_current_pose();
    print "========================================="
    
    print "========= Setting initial pose ================"
    group_variable_values = group.get_current_joint_values()
    group_variable_values[0] = 0
    group_variable_values[1] = 0
    group_variable_values[2] = -2.61
    group_variable_values[3] = 0.99
    group_variable_values[4] = 1.06
    group_variable_values[5] = -1.62
    group_variable_values[6] = 0.4
    group_variable_values[7] = 0
    group_variable_values[8] = 0
    group_variable_values[9] = 0
    group.set_joint_value_target(group_variable_values)
    
    plan_init = group.plan()
    group.go(wait=True)
    print "============ Joint values: ", group_variable_values
    
    
    # Locating the arms and the Kinect2 sensor
    group_left_arm_values = group_left_arm.get_current_joint_values()
    group_right_arm_values = group_right_arm.get_current_joint_values()
    group_kinect_values = group_kinect.get_current_joint_values()
    print "============ End Effector Kinect: %s" % group_kinect.get_end_effector_link()
    
    # Arms in the way of the kinect
    print "========= Setting initial arm pose ================"
    group_left_arm_values[0] = -0.82
    group_left_arm_values[1] = 1.58
    group_left_arm_values[2] = -0.43
    group_left_arm_values[3] = -1.23
    group_left_arm_values[4] = -0.07
    group_left_arm_values[5] = -0.9
    group_left_arm_values[6] =  -0.14

    group_left_arm.set_joint_value_target(group_left_arm_values)
    
    plan_init = group_left_arm.plan()
    group_left_arm.go(wait=True)
    
    # Defining a desired orientation range for the Kinect2
    pose_kinect = geometry_msgs.msg.PoseStamped()
    pose_kinect.header.frame_id = 'base_footprint' # (yo)
    pose_kinect.pose.orientation.x = 0.6
    pose_kinect.pose.orientation.y = -0.6
    pose_kinect.pose.orientation.z = 0.3
    pose_kinect.pose.orientation.w = -0.3
    group_kinect.set_goal_orientation_tolerance(0.5);
    
    ## Planning to a Pose goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    print "============ Generating plan 1"

    pose_target = geometry_msgs.msg.PoseStamped()
    pose_target.header.frame_id = 'base_footprint' # (yo)
    pose_target.pose.orientation.x = 0 #-0.609 # (yo)
    pose_target.pose.orientation.y = 0 #0.609 # (yo)
    pose_target.pose.orientation.z = 0 #-0.387 # (yo)
    pose_target.pose.orientation.w = 1 #0.327  # (yo)
    pose_target.pose.position.x =  1.3 #1.1
    pose_target.pose.position.y = -0.2 #-0.1
    pose_target.pose.position.z = 1.17 #1.05
    
#     x=0
#     while True:
#         group.set_pose_target(pose_target)
# #        plan_target = group.plan()
# 
# #        group.go(wait=True)
#          
# #        print "============ Orientation: ", pose_kinect.pose.orientation
#         print "============ Orientation: ", group_kinect.get_current_pose()
        
        # while True:
        #     if pose_kinect.pose.orientation.z < -0.3 or pose_kinect.pose.orientation.z > 0.6:
        #         group.set_pose_target(pose_target)
        #         group_kinect.get_current_pose()
        #         print "============ Value of x: ", x
        #         rospy.sleep(3)
        #         x=x+1                
        #         if x>5: break
        #     else: break
        
            
       #  group.go(wait=True)
       #  
       #  print "============ Waiting while RVIZ displays plan1"
       #  rospy.sleep(3)
       # 
       # # group.go(wait=True)
       # 
       # 
       #  group_variable_values = group.get_current_joint_values()
       #  print "============ Joint values: ", group_variable_values
       #  print "============ Value of x: ", x
       #  
       #  # Update arms position
       #  group_left_arm_values = group_left_arm.get_current_joint_values()
       #  group_right_arm_values = group_right_arm.get_current_joint_values()

    group.set_pose_target(pose_target)
    print "============ Waiting while RVIZ displays plan1"
    rospy.sleep(3)
    
    group.go(wait=True)
    
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
