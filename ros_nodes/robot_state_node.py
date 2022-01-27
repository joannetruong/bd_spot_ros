from bd_spot_wrapper.spot import (
    Spot,
)
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped

POSE_TOPIC = "/spot_pose"
VEL_TOPIC = "/spot_velocity"

MIN_DEPTH = 0.3*1000
MAX_DEPTH = 10.0*1000

IMG_HEIGHT=480
IMG_WIDTH=640

class RoboStateNode:
    def __init__(self, spot, visualize=False):
        rospy.init_node("camera_node")
        self.spot = spot

        # For generating Image ROS msgs
        self.cv_bridge = CvBridge()

        # Instantiate ROS topic subscribers
        self.state_pub = rospy.Publisher(VIZ_TOPIC, Image, queue_size=5)

        self.pose_pub = rospy.Publisher(POSE_TOPIC, PoseStamped, queue_size=5)
        self.vel_pub = rospy.Publisher(VEL_TOPIC, TwistStamped, queue_size=5)

    def publish_robot_pose(self):
        robot_position = self.spot.get_robot_position()
        robot_quat = self.spot.get_robot_quat()

        robot_pose = PoseStamped()
        robot_pose.header.stamp = rospy.Time.now()
        robot_pose.point.x = robot_position[0]  
        robot_pose.point.y = robot_position[1]  
        robot_pose.point.z = robot_position[2]  
        robot_pose.orientation.x = robot_quat[0]  
        robot_pose.orientation.y = robot_quat[1]  
        robot_pose.orientation.z = robot_quat[2]  
        robot_pose.orientation.w = robot_quat[3] 
        self.pose_pub.publish(robot_pose)

    def publish_robot_vel(self):
        robot_velocity = self.spot.get_robot_vel()

        robot_twist = TwistStamped()
        robot_pose.header.stamp = rospy.Time.now()
        robot_twist.linear.x = vel[0]
        robot_twist.linear.y = vel[1]
        robot_twist.linear.z = vel[2]
        robot_twist.angular.x = vel[0]
        robot_twist.angular.y = vel[1]
        robot_twist.angular.z = vel[2]
        self.vel_pub.publish(robot_twist)

    def publish_robot_state(self):
        self.publish_robot_pose()
        self.publish_robot_vel()


def main():
    spot = Spot("StateROS")
    rsn = RoboStateNode(spot)
    while not rospy.is_shutdown():
        rsn.publish_robot_state()

if __name__ == '__main__':
    main()