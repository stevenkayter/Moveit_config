# Moveit_config

This package contains the files required to control the UR3 on boxy's head using Moveit. 

- boxy_description_urdf contains the geometric representation of boxy and the UR3, including some dummy links used to position the Kinect 2.

- boxy_moveit_config includes the kinematic representation and the general configuration needed by Moveit, as well as the publisher and subscriber used to specify the target of the Kinect2 (where the Kinect 2 is looking).

To run it:
  - Clone this repository and iai_robots
  - bring up the ur3  'roslaunch ur_modern_driver ur3_bringup.launch  robot_ip:=192.168.102.62 prefix:=neck_' 
  - launch move_group_mine.launch, if you want to publish the desired pose, comment the 'node name="move_group_controller" pkg="boxy_moveit_config" type="kinect_controller.py" respawn="false" output="screen"/' part
  - launch kinect_planner.py

To publish your pose, send a PoseStamped to /desired_pose topic with pose_target.header.frame_id = 'map'
