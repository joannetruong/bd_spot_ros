import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image

from spot_wrapper.spot import Spot, SpotCamIds, image_response_to_cv2

FL_TOPIC = "/spot_fl_fisheye"
FR_TOPIC = "/spot_fr_fisheye"
L_TOPIC = "/spot_l_fisheye"
R_TOPIC = "/spot_r_fisheye"
B_TOPIC = "/spot_b_fisheye"

FL_D_TOPIC = "/spot_fl_depth"
FR_D_TOPIC = "/spot_fr_depth"
L_D_TOPIC = "/spot_l_depth"
R_D_TOPIC = "/spot_r_depth"
B_D_TOPIC = "/spot_b_depth"


class SpotCamNode:
    def __init__(self, spot, visualize=True):
        rospy.init_node("spot_cam_node")
        self.spot = spot

        # For generating Image ROS msgs
        self.cv_bridge = CvBridge()

        self.fl_d_pub = rospy.Publisher(FL_D_TOPIC, Image, queue_size=1)
        self.fr_d_pub = rospy.Publisher(FR_D_TOPIC, Image, queue_size=1)
        self.l_d_pub = rospy.Publisher(L_D_TOPIC, Image, queue_size=1)
        self.r_d_pub = rospy.Publisher(R_D_TOPIC, Image, queue_size=1)
        self.b_d_pub = rospy.Publisher(B_D_TOPIC, Image, queue_size=1)

        # Instantiate ROS topic publishers
        self.fl_pub_c = rospy.Publisher(
            FL_TOPIC + "/compressed", CompressedImage, queue_size=1
        )
        self.fr_pub_c = rospy.Publisher(
            FR_TOPIC + "/compressed", CompressedImage, queue_size=1
        )
        self.l_pub_c = rospy.Publisher(
            L_TOPIC + "/compressed", CompressedImage, queue_size=1
        )
        self.r_pub_c = rospy.Publisher(
            R_TOPIC + "/compressed", CompressedImage, queue_size=1
        )
        self.b_pub_c = rospy.Publisher(
            B_TOPIC + "/compressed", CompressedImage, queue_size=1
        )

    def publish_imgs(self):
        image_responses = self.spot.get_image_responses(
            [
                SpotCamIds.FRONTLEFT_FISHEYE,
                SpotCamIds.FRONTRIGHT_FISHEYE,
                SpotCamIds.LEFT_FISHEYE,
                SpotCamIds.RIGHT_FISHEYE,
                SpotCamIds.BACK_FISHEYE,
                SpotCamIds.FRONTLEFT_DEPTH,
                SpotCamIds.FRONTRIGHT_DEPTH,
                SpotCamIds.LEFT_DEPTH,
                SpotCamIds.RIGHT_DEPTH,
                SpotCamIds.BACK_DEPTH,
            ]
        )
        # convert images to cv2 and rotate images ccw 90
        (
            fl_img,
            fr_img,
            l_img,
            r_img,
            b_img,
            fl_d_img,
            fr_d_img,
            l_d_img,
            r_d_img,
            b_d_img,
        ) = [np.rot90(image_response_to_cv2(i), k=3) for i in image_responses]

        # publish front left grayscale img
        fl_img_msg_c = self.cv_bridge.cv2_to_compressed_imgmsg(fl_img)
        self.fl_pub_c.publish(fl_img_msg_c)

        # publish front right grayscale img
        fr_img_msg_c = self.cv_bridge.cv2_to_compressed_imgmsg(fr_img)
        self.fr_pub_c.publish(fr_img_msg_c)

        # publish left grayscale img
        l_img_msg_c = self.cv_bridge.cv2_to_compressed_imgmsg(l_img)
        self.l_pub_c.publish(l_img_msg_c)

        # publish right grayscale img
        r_img_msg_c = self.cv_bridge.cv2_to_compressed_imgmsg(r_img)
        self.r_pub_c.publish(r_img_msg_c)

        # publish back grayscale img
        b_img_msg_c = self.cv_bridge.cv2_to_compressed_imgmsg(b_img)
        self.b_pub_c.publish(b_img_msg_c)

        # publish front left depth img
        fl_d_img_msg = self.cv_bridge.cv2_to_imgmsg(fl_d_img, encoding="mono16")
        self.fl_d_pub.publish(fl_d_img_msg)

        # publish front right depth img
        fr_d_img_msg = self.cv_bridge.cv2_to_imgmsg(fr_d_img, encoding="mono16")
        self.fr_d_pub.publish(fr_d_img_msg)

        # publish left depth img
        l_d_img_msg = self.cv_bridge.cv2_to_imgmsg(l_d_img, encoding="mono16")
        self.l_d_pub.publish(l_d_img_msg)

        # publish right depth img
        r_d_img_msg = self.cv_bridge.cv2_to_imgmsg(r_d_img, encoding="mono16")
        self.r_d_pub.publish(r_d_img_msg)

        # publish back depth img
        b_d_img_msg = self.cv_bridge.cv2_to_imgmsg(b_d_img, encoding="mono16")
        self.b_d_pub.publish(b_d_img_msg)


def main():
    spot = Spot("SpotCamROS")
    scn = SpotCamNode(spot)
    while not rospy.is_shutdown():
        scn.publish_imgs()


if __name__ == "__main__":
    main()
