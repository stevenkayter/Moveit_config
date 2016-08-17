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

# DESCRIPTION: Publishes the desired pose ot joint values for the Microsotf Kinect2 mounted as head of boxy
import sys
import copy
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from boxy_moveit_config.msg import pose_w_joints

def kinect_pose_definition():
    rospy.sleep(10)
    rospy.init_node('robot_state', anonymous=True,)
    rate = rospy.Rate(0.5)

    # We create this publisher for the desired pose of the Kinect2 Sensor
    pose_publisher = rospy.Publisher("desired_pose", PoseStamped, queue_size=5)
    joint_publisher = rospy.Publisher("desired_joints", pose_w_joints, queue_size=5)

    # Defining a Pose goal (wrt base)
    pose_target = PoseStamped()
    pose_target.pose.orientation.x = 0 
    pose_target.pose.orientation.y = 0 
    pose_target.pose.orientation.z = 0 
    pose_target.pose.orientation.w = 1 
    pose_target.header.frame_id = 'base_footprint'
    pose_target.pose.position.x =  1.256
    pose_target.pose.position.y = -0.152
    pose_target.pose.position.z = 0.853

    # Defining a Pose goal (wrt map, when Boxy is up)
    #pose_target.header.frame_id = 'map'
    #pose_target.pose.position.x =  -1.159
    #pose_target.pose.position.y = 1.42
    #pose_target.pose.position.z = 0.85
    print "Pose target: ", pose_target

    # Defining joints desired values
    neck_joints = [0,0,0,0,0,0,0,0,0,0]
    neck_joints[0] = -1.346
    neck_joints[1] = -1.116
    neck_joints[2] = -2.121
    neck_joints[3] = 0.830
    neck_joints[4] = 1.490
    neck_joints[5] = 0.050
    neck_joints[6] = 0
    neck_joints[7] = 0
    neck_joints[8] = 0
    neck_joints[9] = 0

    while not rospy.is_shutdown():

        # Publish desired pose
        #pose_publisher.publish(pose_target)
        # Publish desired joint state
        joint_publisher.publish(neck_joints)

        rate.sleep()


if __name__=='__main__':
    try:
        kinect_pose_definition()
    except rospy.ROSInterruptException:
        pass
