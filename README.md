# bd_spot_ros

Python wrapper for Boston Dynamics Spot

```
pip install -e . && \
pip install -r requirements.txt
```

## Publish Spot's camera images to ROS:
`python ros_nodes/spot_camera_node.py`

## Publish Spot's state [pose, velocity, joint_states] to ROS:
`python ros_nodes/robot_state_node.py`

## Record ROS bag:
`rosbag record -o spot_img_state_run_1.bag /spot_joint_states /spot_odom_vel /spot_pose /spot_vision_vel /spot_fl_fisheye /spot_fr_fisheye /spot_fl_depth /spot_fr_depth`

## Playback data:
`rosbag play <rosbag_name.bag>`

`rosrun rviz rviz -d configs/spot_img_pose.rviz`


#### Acknowledgments
Adapted from Naoki Yokoyama