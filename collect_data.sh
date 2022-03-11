#!/usr/bin/env bash

killall -9 roscore
killall -9 rosmaster
roscore & \
python ros_nodes/spot_camera_node.py & \
python ros_nodes/spot_state_node.py & \
rosbag record -o bags/spot_cam_state.bag /spot_joint_states /spot_odom_vel /spot_pose /spot_vision_vel /spot_fl_fisheye/compressed /spot_fr_fisheye/compressed /spot_l_fisheye/compressed /spot_r_fisheye/compressed /spot_b_fisheye/compressed /spot_fl_depth /spot_fr_depth /spot_l_depth /spot_r_depth /spot_b_depth /spot_fr_foot_contact /spot_fl_foot_contact /spot_rr_foot_contact /spot_rl_foot_contact /spot_fr_foot_position /spot_fl_foot_position /spot_rr_foot_position /spot_rl_foot_position /spot_fr_ground_mu /spot_fl_ground_mu /spot_rr_ground_mu /spot_rl_ground_mu