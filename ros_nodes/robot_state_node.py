from spot_wrapper.spot import (
    Spot,
)
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Path

POSE_TOPIC = "/spot_pose"
PATH_TOPIC = "/spot_path"
VIS_VEL_TOPIC = "/spot_vision_vel"
ODOM_VEL_TOPIC = "/spot_odom_vel"
JOINT_STATE_TOPIC = "/spot_joint_states"

class RoboStateNode:
    def __init__(self, spot, visualize=False):
        rospy.init_node("camera_node")
        self.spot = spot

        # Instantiate ROS topic subscribers
        self.path_pub = rospy.Publisher(PATH_TOPIC, Path, queue_size=5)
        self.pose_pub = rospy.Publisher(POSE_TOPIC, PoseStamped, queue_size=5)
        self.vis_vel_pub = rospy.Publisher(VIS_VEL_TOPIC, TwistStamped, queue_size=5)
        self.odom_vel_pub = rospy.Publisher(ODOM_VEL_TOPIC, TwistStamped, queue_size=5)
        self.joint_state_pub = rospy.Publisher(JOINT_STATE_TOPIC, JointState, queue_size=5)

    def publish_robot_pose(self):
        robot_position = self.spot.get_robot_position()
        robot_quat = self.spot.get_robot_quat()

        robot_pose = PoseStamped()
        robot_pose.header.stamp = rospy.Time.now()
        robot_pose.header.frame_id = 'map'
        robot_pose.pose.position.x = robot_position[0]  
        robot_pose.pose.position.y = robot_position[1]  
        robot_pose.pose.position.z = robot_position[2]  
        robot_pose.pose.orientation.x = robot_quat[0]  
        robot_pose.pose.orientation.y = robot_quat[1]  
        robot_pose.pose.orientation.z = robot_quat[2]  
        robot_pose.pose.orientation.w = robot_quat[3] 
        self.pose_pub.publish(robot_pose)

    def vel_to_twist_msg(self, frame):
        robot_linear_velocity = self.spot.get_robot_linear_vel(frame)
        robot_angular_velocity = self.spot.get_robot_angular_vel(frame)

        robot_twist = TwistStamped()
        robot_twist.header.stamp = rospy.Time.now()
        robot_twist.twist.linear.x = robot_linear_velocity[0]
        robot_twist.twist.linear.y = robot_linear_velocity[1]
        robot_twist.twist.linear.z = robot_linear_velocity[2]
        robot_twist.twist.angular.x = robot_angular_velocity[0]
        robot_twist.twist.angular.y = robot_angular_velocity[1]
        robot_twist.twist.angular.z = robot_angular_velocity[2]
        return robot_twist

    def publish_robot_vel(self):
        robot_vis_twist = self.vel_to_twist_msg('vision')
        robot_odom_twist = self.vel_to_twist_msg('odom')

        self.vis_vel_pub.publish(robot_vis_twist)
        self.odom_vel_pub.publish(robot_odom_twist)

    def publish_robot_joint_states(self):
        robot_joint_states = JointState()
        joint_states = self.spot.get_robot_joint_states()

        all_data = [
            (name, data.position.value, data.velocity.value, data.acceleration.value)
            for name, data in joint_states.items()
        ]
        names, positions, velocities, acceleration = [list(x) for x in zip(*all_data)]

        robot_joint_states.name = names
        robot_joint_states.position = positions
        robot_joint_states.velocity = velocities
        robot_joint_states.effort = acceleration

        self.joint_state_pub.publish(robot_joint_states)

    def publish_robot_feet_state(self):
        feet_state = self.spot.get_robot_foot_state()
        # print('JOANNE: ', feet_state)

    def publish_robot_state(self):
        self.publish_robot_pose()
        self.publish_robot_vel()
        self.publish_robot_feet_state()
        self.publish_robot_joint_states()


def main():
    spot = Spot("StateROS")
    rsn = RoboStateNode(spot)
    while not rospy.is_shutdown():
        rsn.publish_robot_state()

if __name__ == '__main__':
    main()