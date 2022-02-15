# bd_spot_ros

Python wrapper for Boston Dynamics Spot

```
pip install -e . && \
pip install -r requirements.txt
```
## Collect data:
To collect camera and pose data from Spot, run:

`./collect_data.sh`

This bash file runs multiple scripts at once:
* `roscore` [Start ROS for communicating between nodes]
* `python ros_nodes/spot_camera_node.py` [Publish Spot's camera images to ROS]
* `python ros_nodes/spot_state_node.py` [Publish Spot's pose, velocity, joint_states to ROS]
* `rosbag record -o bags/tmp.bag /spot_joint_states /spot_odom_vel /spot_pose /spot_vision_vel /spot_fl_fisheye/compressed /spot_fr_fisheye/compressed /spot_l_fisheye/compressed /spot_r_fisheye/compressed /spot_b_fisheye/compressed /spot_fl_depth /spot_fr_depth /spot_l_depth /spot_r_depth /spot_b_depth` [Save data from these rostopics]

## Playback data:
`rosbag play <rosbag_name.bag>`

`rosrun rviz rviz -d configs/spot_img_pose.rviz`

## External Camera:
If using an external camera (Intel Realsense Camera), you can launch the camera node via:
`roslaunch realsense2_camera rs_aligned_depth.launch`

and record data using:
`rosbag record -o bags/spot_intel_state_run_1.bag /spot_joint_states /spot_odom_vel /spot_pose /spot_vision_vel /camera/color/image_raw /camera/aligned_depth_to_color/image_raw`
#### Acknowledgments
Adapted from Naoki Yokoyama