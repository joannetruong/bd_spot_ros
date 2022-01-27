from spot_wrapper.spot import (
    Spot,
    SpotCamIds,
    image_response_to_cv2,
)
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image

FL_TOPIC = "/spot_fl_fisheye"
FR_TOPIC = "/spot_fr_fisheye"

FL_D_TOPIC = "/spot_fl_depth"
FR_D_TOPIC = "/spot_fr_depth"

class SpotCamNode:
    def __init__(self, spot, visualize=True):
        rospy.init_node("spot_cam_node")
        self.spot = spot

        # For generating Image ROS msgs
        self.cv_bridge = CvBridge()

        # Instantiate ROS topic publishers
        self.fl_pub = rospy.Publisher(FL_TOPIC, Image, queue_size=5)
        self.fr_pub = rospy.Publisher(FR_TOPIC, Image, queue_size=5)

        self.fl_d_pub = rospy.Publisher(FL_D_TOPIC, Image, queue_size=5)
        self.fr_d_pub = rospy.Publisher(FR_D_TOPIC, Image, queue_size=5)d_

    def publish_imgs(self):
        image_responses = self.spot.get_image_responses([SpotCamIds.FRONTLEFT_FISHEYE, SpotCamIds.FRONTRIGHT_FISHEYE])
        fl_img, fr_img = [image_response_to_cv2(i) for i in image_responses]

        fl_img_msg = self.cv_bridge.cv2_to_imgmsg(fl_img, "mono8")
        self.fl_pub.publish(fl_img_msg)

        fr_img_msg = self.cv_bridge.cv2_to_imgmsg(fr_img, "mono8")
        self.fr_pub.publish(fr_img_msg)


def main():
    spot = Spot("SpotCamROS")
    scn = SpotCamNode(spot)
    while not rospy.is_shutdown():
        scn.publish_imgs()

if __name__ == '__main__':
    main()