# Moveit_config

This package contains the files required to control the UR3 on boxy's head using Moveit. It can be controlled by sending a pose or a joint state.

- `boxy_moveit_config` includes the kinematic representation and the general configuration needed by Moveit, as well as the publisher and subscriber used to specify the target of the Kinect2 (where the Kinect 2 is looking). It uses the URDF models of iai_robots.

To run it you must clone __this__ repository, __iai_boxy__ and __iai_robots__. The main consideration is checking if boxy is already up or not.

### If boxy is *down*:
  - In __move_neck.launch__ from the package boxy_moveit_config, set the argument `bring_up_robot` to `default="true"`.
  - Bring up the ur3:

        `roslaunch ur_modern_driver ur3_bringup.launch  robot_ip:=192.168.102.62 prefix:=neck_`

  - Launch __move_neck.launch__.
  
        ` roslaunch boxy_moveit_config move_neck.launch robot_ip:=192.168.102.62 `

  This includes the __kinect_planner.py__ from the package boxy_moveit_config, which suscribes to `/desired_pose` and `/desired_joints` to read the target pose or joint state.
  
  - __Optional:__ If you want to visualize the robot, launch 
        ` roslaunch boxy_moveit_config moveit_rviz.launch `

### If boxy is already *up*:
  - In `move_neck.launch`: modify `arg name="bring_up_robot"` to `default="false"`
  - Launch __move_neck.launch__.
  
        ` roslaunch boxy_moveit_config move_neck.launch robot_ip:=192.168.102.62 `

  - __Optional:__ If you want to visualize the robot, launch 
        ` roslaunch boxy_moveit_config moveit_rviz.launch 


## Publish your desired pose or joint state

To __publish your pose__, send a PoseStamped to `/desired_pose` topic with `pose_target.header.frame_id = 'map'`. 
Example: 

   ```
  rostopic pub /desired_pose geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'base_footprint'
pose:
  position:
    x: 1.256
    y: -0.152
    z: 0.853
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0"  --once
    ```
    __NOTE:__ The `frame_id` should be set to `base_footprint` when boxy is down to and `map` when it's up.
    
To publish some desired joint values, send a pose_w_joint message to `/desired_joints` (`from boxy_moveit_config.msg import pose_w_joints`). Example:

  ```
  rostopic pub /desired_joints boxy_moveit_config/pose_w_joints "joint_values:
 [-1.3459999561309814, -1.1160000562667847, -2.121000051498413, 0.8299999833106995, 1.4900000095367432, 0.05000000074505806, 0.0, 0.0, 0.0, 0.0]" --once
  ```
  
An example of a pose and a joint state being published is in `kinect_controller.py`:
        ` rosrun boxy_moveit_config kinect_controller.py `



