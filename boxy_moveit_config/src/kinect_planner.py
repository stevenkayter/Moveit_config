#!/usr/bin/env python
# Copyright (c) 2016 Universitaet Bremen - Institute for Artificial Intelligence (Prof. Beetz)
#
# Author: Minerva Gabriela Vargas Gleason mvargasg@uni-bremen.de
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

# DESCRIPTION:Controls the position of the Microsotf Kinect2 mounted as head of boxy

import sys
import copy
import rospy
import actionlib
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import OrientationConstraint
from moveit_msgs.msg import CollisionObject
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from control_msgs.msg import *
from trajectory_msgs.msg import *
import roslib; roslib.load_manifest('ur_driver')
from boxy_moveit_config.msg import pose_w_joints

fresh_data = False
pose_target = PoseStamped()
pose_req = False
joints_req = False

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',  'joint_end', 'head1',
               'head2', 'head3', 'triangle_base_joint', 'left_arm_0_joint', 'left_arm_1_joint',
               'left_arm_2_joint', 'left_arm_3_joint', 'left_arm_4_joint', 'left_arm_5_joint',
               'left_arm_6_joint', 'left_gripper_base_gripper_left_joint',
               'left_gripper_base_gripper_right_joint', 'right_arm_0_joint', 'right_arm_1_joint',
               'right_arm_2_joint', 'right_arm_3_joint', 'right_arm_4_joint', 'right_arm_5_joint',
               'right_arm_6_joint','right_gripper_base_gripper_left_joint',
               'right_gripper_base_gripper_right_joint']

client = None

goal=FollowJointTrajectoryGoal()

def callback(data):
    global pose_target
    global fresh_data, pose_req
    pose_target = data   
    fresh_data = True
    pose_req = True # Flag to se if target is pose or joints

def callback_joints(data):
    global joint_target
    global fresh_data, joints_req
    joint_target = list(data.joint_values)
    fresh_data = True
    joints_req = True # Flag to se if target is pose or joints

def move(plan_target):
    global goal
    goal.trajectory = plan_target.joint_trajectory
    #goal.trajectory.joint_names = JOINT_NAMES

def kinect_planner():
    global fresh_data
    global joint_target, pose_target, goal
    global joints_req, pose_req
    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('kinect_trajectory_planner', anonymous=True)
    rate = rospy.Rate(0.5)
    #rospy.sleep(3)
       
    # Instantiate a RobotCommander object. 
    robot = moveit_commander.RobotCommander()
    
    # Instantiate a PlanningSceneInterface object (interface to the world surrounding the robot).
    scene = moveit_commander.PlanningSceneInterface()
    print "===================== Here 1 ======================="
    
    # Instantiate MoveGroupCommander objects for arms and Kinect2. 
    group = moveit_commander.MoveGroupCommander("Kinect2_Target")
    group_left_arm = moveit_commander.MoveGroupCommander("left_arm")
    group_right_arm = moveit_commander.MoveGroupCommander("right_arm")

    # We create this DisplayTrajectory publisher to publish
    # trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher('planned_path',
                                        moveit_msgs.msg.DisplayTrajectory, queue_size=5)

    # Set the planner for Moveit
    group.set_planner_id("RRTConnectkConfigDefault")
    group.set_planning_time(8)
    group.set_pose_reference_frame('base_footprint')
    
    # Setting tolerance
    group.set_goal_tolerance(0.08)
    group.set_num_planning_attempts(10)
    
    # Suscribing to the desired pose topic
    target = rospy.Subscriber("desired_pose", PoseStamped, callback)
    target_joint = rospy.Subscriber("desired_joints", pose_w_joints, callback_joints)
    
    # Locating the arms and the Kinect2 sensor
    group_left_arm_values = group_left_arm.get_current_joint_values()
    group_right_arm_values = group_right_arm.get_current_joint_values()
    #group_kinect_values = group_kinect.get_current_joint_values()

    neck_init_joints = group.get_current_joint_values()
    neck_init_joints[0] = -1.346
    neck_init_joints[1] = -1.116
    neck_init_joints[2] = -2.121
    neck_init_joints[3] = 0.830
    neck_init_joints[4] = 1.490
    neck_init_joints[5] = 0.050
    neck_init_joints[6] = 0
    neck_init_joints[7] = 0
    neck_init_joints[8] = 0
    neck_init_joints[9] = 0

    # Creating a box to limit the arm position
    box_pose = PoseStamped()
    box_pose.pose.orientation.w = 1
    box_pose.pose.position.x =  0.6
    box_pose.pose.position.y =  0.03
    box_pose.pose.position.z =  1.5
    box_pose.header.frame_id = 'base_footprint'

    scene.add_box('box1', box_pose, (0.4, 0.4, 0.1))
    rospy.sleep(2)

    # Defining position constraints for the trajectory of the kinect
    neck_const = Constraints()
    neck_const.name = 'neck_constraints'
    target_const = JointConstraint()
    target_const.joint_name = "neck_joint_end"
    target_const.position = 0.95
    target_const.tolerance_above = 0.45
    target_const.tolerance_below = 0.05
    target_const.weight = 0.9
    neck_const.joint_constraints.append(target_const)

    kinect_const = OrientationConstraint()
    kinect_const.header.frame_id = "base_footprint"
    kinect_const.link_name = "neck_tool0"
    kinect_const.orientation.x =  0.373
    kinect_const.orientation.y = -0.361
    kinect_const.orientation.z =  0.630
    kinect_const.orientation.w = -0.578
    kinect_const.absolute_x_axis_tolerance = 3.14
    kinect_const.absolute_y_axis_tolerance = 0.450
    kinect_const.absolute_z_axis_tolerance = 3.14
    kinect_const.weight = 0.7 # Importance of this constraint
    neck_const.orientation_constraints.append(kinect_const)
    #group.set_path_constraints(neck_const)

    # Talking to the robot
    client = actionlib.SimpleActionClient('/Kinect2_Target_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    print "====== Waiting for server..."
    client.wait_for_server()
    print "====== Connected to server"
    
    while not rospy.is_shutdown():
        if fresh_data == True: #If a new pose is received, plan.
        
            # Update arms position
            group_left_arm_values = group_left_arm.get_current_joint_values()
            group_right_arm_values = group_right_arm.get_current_joint_values()
            
            # Set the target pose (or joint values) and generate a plan
            if pose_req:
                group.set_pose_target(pose_target)
            if joints_req:
                group.set_joint_value_target(joint_target)

            print "=========== Calculating trajectory... \n"

            # Generate several plans and compare them to get the shorter one
            plan_opt = dict()
            differ = dict()

            try:
                num = 1
                rep = 0

                # Generate several plans and compare them to get the shortest one
                #for num in range(1,8):
                while num < 7:
                    num += 1
                    plan_temp = group.plan()
                    move(plan_temp)
                    plan_opt[num] = goal.trajectory
                    diff=0
                    for point in goal.trajectory.points:
                        # calculate the distance between initial pose and planned one
                        for i in range(0,6):
                            diff = abs(neck_init_joints[i] - point.positions[i])+abs(diff)
                        differ[num] = diff
                    # If the current plan is good, take it
                    if diff < 110:
                       break
                    # If plan is too bad, don't consider it
                    if diff > 400:
                        num = num - 1
                        print "Plan is too long. Replanning."
                        rep = rep + 1
                        if rep > 4:
                            num = num +1

                # If no plan was found...
                if differ == {}:
                    print "---- Fail: No motion plan found. No execution attempted. Probably robot joints are out of range."
                    break

                else:
                    # Select the shortest plan
                    min_plan = min(differ.itervalues())
                    select = [k for k, value in differ.iteritems() if value == min_plan]
                    goal.trajectory = plan_opt[select[0]]
                    print " Plan difference:========= ", differ
                    print " Selected plan:=========== ", select[0]

                    # Remove the last 4 names and data from each point (dummy joints) before sending the goal
                    goal.trajectory.joint_names = goal.trajectory.joint_names[:6]
                    for point in goal.trajectory.points:
                        point.positions = point.positions[:6]
                        point.velocities = point.velocities[:6]
                        point.accelerations = point.accelerations[:6]

                    print "Sending goal"
                    client.send_goal(goal)
                    print "Waiting for result"
                    client.wait_for_result()

                    # Change the position of the virtual joint to avoid collision
                    neck_joints = group.get_current_joint_values()
                    #neck_joints[6] = 0.7
                    #group.set_joint_value_target(neck_joints)
                    #group.go(wait=True)


            except (KeyboardInterrupt, SystemExit):
                client.cancel_goal()
                raise
            
            rate.sleep()
    
        fresh_data = False
    
        rate.sleep()




if __name__=='__main__':
  try:
    kinect_planner()
  except rospy.ROSInterruptException:
    pass
