#!/usr/bin/env bash

roscore & \
python ros_nodes/spot_camera_node.py & \
python ros_nodes/robot_state_node.py & \
rosbag record -o bags/tmp.bag /spot_joint_states /spot_odom_vel /spot_pose /spot_vision_vel /spot_fl_fisheye/compressed /spot_fr_fisheye/compressed /spot_fl_depth /spot_fr_depth