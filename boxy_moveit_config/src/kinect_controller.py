#!/usr/bin/env python

import sys
import copy
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

def kinect_pose_definition():
    rospy.sleep(10)
    rospy.init_node('robot_state', anonymous=True,)
    rate = rospy.Rate(0.5)

    # We create this publisher for the desired pose of the Kinect2 Sensor
    pose_publisher = rospy.Publisher("desired_pose", PoseStamped, queue_size=5)

    # Defining a Pose goal (wrt base)
    pose_target = PoseStamped()
    pose_target.pose.orientation.x = 0 
    pose_target.pose.orientation.y = 0 
    pose_target.pose.orientation.z = 0 
    pose_target.pose.orientation.w = 1 
    #pose_target.header.frame_id = 'base_footprint'
    #pose_target.pose.position.x =  1.256
    #pose_target.pose.position.y = -0.152
    #pose_target.pose.position.z = 0.853

    # Defining a Pose goal (wrt map, when Boxy is up)
    pose_target.header.frame_id = 'map'
    pose_target.pose.position.x =  -1.159
    pose_target.pose.position.y = 1.42
    pose_target.pose.position.z = 0.85
    print "Pose target: ", pose_target


    while not rospy.is_shutdown():

        # Publish desired pose
        pose_publisher.publish(pose_target)

        rate.sleep()


if __name__=='__main__':
    try:
        kinect_pose_definition()
    except rospy.ROSInterruptException:
        pass
