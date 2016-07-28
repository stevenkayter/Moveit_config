#!/usr/bin/env python

import sys
import copy
import rospy
import actionlib
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from control_msgs.msg import *
from trajectory_msgs.msg import *
import roslib; roslib.load_manifest('ur_driver')

fresh_data = False
pose_target = PoseStamped()

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
    global fresh_data
    pose_target = data   
    fresh_data = True

def move(plan_target):
    global goal
    goal.trajectory = plan_target.joint_trajectory
    #goal.trajectory.joint_names = JOINT_NAMES

def kinect_planner():
    global fresh_data
    global goal
    
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
    #group_kinect = moveit_commander.MoveGroupCommander("neck")

    # We create this DisplayTrajectory publisher to publish
    # trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher('planned_path',
                                        moveit_msgs.msg.DisplayTrajectory, queue_size=5)

    # Set the planner for Moveit
    group.set_planner_id("RRTConnectkConfigDefault")
    group.set_planning_time(10)
    group.set_pose_reference_frame('base_footprint')
    
    # Setting tolerance
    group.set_goal_tolerance(0.08)
    group.set_num_planning_attempts(10)
    
    #print "=============== Current Pose ==========="
    #print group.get_current_pose();
    
    # Suscribing to the desired pose topic
    target = rospy.Subscriber("desired_pose", PoseStamped, callback)
    
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
            
            # Set the target pose and generate a plan
            group.set_pose_target(pose_target)
            #plan_target = group.plan()
            print "=========== Calculating trajectory... \n"

            # Generate several plans and compare them to get the shorter one
            plan_opt = dict()
            differ = dict()

            try:
                #group.go(wait=True)
                #move(plan_target)
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
                    if 5 < diff < 110:
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
                    print " Plan difference:=========== ", differ
                    print " Selected plan:============= ", select[0]

                    # Just for testing, remove later
                    diff=0
                    for point in goal.trajectory.points:
                        for i in range(0,6):
                            diff = abs(neck_init_joints[i] - point.positions[i])+abs(diff)
                    print " Selected plan diff:======= ", diff


                    # Remove the last 4 names and data from each point (dummy joints) before sending the goal
                    goal.trajectory.joint_names = goal.trajectory.joint_names[:6]
                    #print "Goal shot", goal
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
                    neck_joints[6] = 0.7
                    group.set_joint_value_target(neck_joints)
                    group.go(wait=True)

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
