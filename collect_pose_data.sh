#!/usr/bin/env bash

killall -9 roscore
killall -9 rosmaster
roscore & \
python ros_nodes/spot_state_node.py & \
rosbag record -o bags/spot_state.bag /spot_joint_states /spot_odom_vel /spot_pose /spot_vision_vel /spot_fr_foot_contact /spot_fl_foot_contact /spot_rr_foot_contact /spot_rl_foot_contact /spot_fr_foot_position /spot_fl_foot_position /spot_rr_foot_position /spot_rl_foot_position /spot_fr_ground_mu /spot_fl_ground_mu /spot_rr_ground_mu /spot_rl_ground_mu