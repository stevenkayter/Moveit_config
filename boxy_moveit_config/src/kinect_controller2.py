#!/usr/bin/env python

import sys
import copy
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

def kinect_pose_definition():
    rospy.sleep(7)
    rospy.init_node('robot_state', anonymous=True,)
    rate = rospy.Rate(1)
    # We create this publisher for the desired pose of the Kinect2 Sensor
    pose_publisher = rospy.Publisher("desired_pose", PoseStamped, queue_size=5)
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
    print "Pose target", pose_target
    x=0;
    while not rospy.is_shutdown():
        if x > 15:
            pose_target.pose.position.x =  1 
            pose_target.pose.position.y = -0.4 
            pose_target.pose.position.z = 1.4
        # Publish desired pose
        pose_publisher.publish(pose_target)
        x=x+1

        rate.sleep()


if __name__=='__main__':
    try:
        kinect_pose_definition()
    except rospy.ROSInterruptException:
        pass
