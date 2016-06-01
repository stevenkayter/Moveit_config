#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String

def kinect_planner():

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('robot_state_publisher',
                    anonymous=True,)

    # Instantiate a RobotCommander object. 
    robot = moveit_commander.RobotCommander()

    # Instantiate a PlanningSceneInterface object.
    # This object is an interface to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()

    # Instantiate MoveGroupCommander objects for arms and Kinect2. 
    group = moveit_commander.MoveGroupCommander("Kinect2_Target")
    group_left_arm = moveit_commander.MoveGroupCommander("left_arm")
    group_right_arm = moveit_commander.MoveGroupCommander("right_arm")
    group_kinect = moveit_commander.MoveGroupCommander("neck")

    # We create this RobotTrajectory publisher to publish the computed trajectories.
    display_trajectory_publisher = rospy.Publisher(
                                        '/move_group/planned_path',
                                        moveit_msgs.msg.RobotTrajectory, queue_size=5)

    print "============ Waiting for RVIZ..."
    rospy.sleep(8)

    ## We can get the name of the reference frame for this robot
    group.set_pose_reference_frame('base_footprint')
    print "============ Reference frame: %s" % group.get_pose_reference_frame()
    print "============ End Effector: %s" % group.get_end_effector_link()

    ## Set the planner for Moveit 
    group.set_planner_id("RRTConnectkConfigDefault")
#    group.set_planner_id("ESTkConfigDefault")
#    group.set_planner_id("KPIECEkConfigDefault")
#    group.set_planner_id("PRMkConfigDefault")
#    group.set_planner_id("PRMstarkConfigDefault")
#    group.set_planner_id("RRTkConfigDefault")

    ## Setting tolerance 
    group.set_goal_tolerance(0.03)
#    group.set_num_planning_attempts(10)

    print "=============== Current Pose ==========="
    print group.get_current_pose();
    print "========================================="
    
    print "========= Getting initial pose ================"
    group_variable_values = group.get_current_joint_values()
    # group_variable_values[0] = 0
    # group_variable_values[1] = 0
    # group_variable_values[2] = -2.61
    # group_variable_values[3] = 0.99
    # group_variable_values[4] = 1.06
    # group_variable_values[5] = -1.62
    # group_variable_values[6] = 0.4
    # group_variable_values[7] = 0
    # group_variable_values[8] = 0
    # group_variable_values[9] = 0
    # group.set_joint_value_target(group_variable_values)
    # 
    # plan_init = group.plan()
    # group.go(wait=True)
    print "============ Joint values: ", group_variable_values
    
    
    # Locating the arms and the Kinect2 sensor
    group_left_arm_values = group_left_arm.get_current_joint_values()
    group_right_arm_values = group_right_arm.get_current_joint_values()
    group_kinect_values = group_kinect.get_current_joint_values()

    
    ## Planning to a Pose goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    print "============ Generating plan 1"

    pose_target = geometry_msgs.msg.PoseStamped()
    

    while not rospy.is_shutdown():

        # Update arms position
        group_left_arm_values = group_left_arm.get_current_joint_values()
        group_right_arm_values = group_right_arm.get_current_joint_values()
        
        # Set the target pose and generate a plan
        group.set_pose_target(pose_target)
        plan_target = group.plan()

        print "============ Orientation: ", group_kinect.get_current_pose()
        print "============ Waiting while the plan is executed"
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