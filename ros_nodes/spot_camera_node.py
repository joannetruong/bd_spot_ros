from spot_wrapper.spot import (
    Spot,
    SpotCamIds,
    image_response_to_cv2,
)
import cv2
import numpy as np
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import CompressedImage, Image

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

        self.fl_d_pub = rospy.Publisher(FL_D_TOPIC, Image, queue_size=1)
        self.fr_d_pub = rospy.Publisher(FR_D_TOPIC, Image, queue_size=1)

        # Instantiate ROS topic publishers
        self.fl_pub_c = rospy.Publisher(FL_TOPIC + '/compressed', CompressedImage, queue_size=1)
        self.fr_pub_c = rospy.Publisher(FR_TOPIC + '/compressed', CompressedImage, queue_size=1)

    def publish_imgs(self):
        image_responses = self.spot.get_image_responses([SpotCamIds.FRONTLEFT_FISHEYE, SpotCamIds.FRONTRIGHT_FISHEYE, SpotCamIds.FRONTLEFT_DEPTH, SpotCamIds.FRONTRIGHT_DEPTH])
        # convert images to cv2 and rotate images ccw 90
        fl_img, fr_img, fl_d_img, fr_d_img = [np.rot90(image_response_to_cv2(i), k=3) for i in image_responses]

        fl_img_msg_c = self.cv_bridge.cv2_to_compressed_imgmsg(fl_img)
        self.fl_pub_c.publish(fl_img_msg_c)

        fr_img_msg_c = self.cv_bridge.cv2_to_compressed_imgmsg(fr_img)
        self.fr_pub_c.publish(fr_img_msg_c)

        fl_d_img_msg = self.cv_bridge.cv2_to_imgmsg(fl_d_img, encoding='mono16')
        self.fl_d_pub.publish(fl_d_img_msg)

        fl_d_img_msg = self.cv_bridge.cv2_to_imgmsg(fr_d_img, encoding='mono16')
        self.fr_d_pub.publish(fl_d_img_msg)

def main():
    spot = Spot("SpotCamROS")
    scn = SpotCamNode(spot)
    while not rospy.is_shutdown():
        scn.publish_imgs()

if __name__ == '__main__':
    main()