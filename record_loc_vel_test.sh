#!/usr/bin/env bash

roscore & \
python ros_nodes/spot_state_node.py & \
rosbag record -o bags/spot_state.bag /spot_odom_vel /spot_pose & \
python spot_wrapper/localization_test.py -v
