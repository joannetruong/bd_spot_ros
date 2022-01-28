from bd_spot_wrapper.spot import (
    Spot,
    SpotCamIds,
    image_response_to_cv2,
)
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image, CompressedImage

RGB_TOPIC = "/camera/color/image_raw/compressed"
DEPTH_TOPIC = "/camera/aligned_depth_to_color/image_raw/compressed"

MIN_DEPTH = 0.3*1000
MAX_DEPTH = 10.0*1000

IMG_HEIGHT=480
IMG_WIDTH=640

class CameraNode:
    def __init__(self, spot, visualize=False):
        rospy.init_node("camera_node")
        self.spot = spot

        # For generating Image ROS msgs
        self.cv_bridge = CvBridge()

        # Instantiate ROS topic subscribers
        self.rgb_sub = rospy.Subscriber(RGB_TOPIC, CompressedImage, self.RGBCallback)
        self.depth_sub = rospy.Subscriber(DEPTH_TOPIC, CompressedImage, self.DepthCallback)

    def ros2numpy(self, data, ty):
        np_arr = np.fromstring(data.data, np.uint16)
        num_channels = 1
        if ty == 'depth'
            np_arr = np.clip(np_arr, a_min=MIN_DEPTH, a_max=MAX_DEPTH)
            np_arr = (np_arr - MIN_DEPTH)/MAX_DEPTH
            num_channels = 3

        image_np = np.reshape(np_arr, [IMG_HEIGHT, IMG_WIDTH, num_channels])
        if ty == 'depth':
            image_np = pillow.fromarray(np.squeeze(image_np))
        else:
            image_np = pillow.fromarray(np.squeeze(image_np), mode='RGB')
        newsize = (self.img_width, self.img_height)
        image_np = image_np.resize(newsize)
        return = np.asanyarray(image_np)

    def DepthCallback(self, data):
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = data.data
        # Publish new image
        self.image_pub.publish(msg)

        if self.visualize:
            msg = msg_Image()
            msg.header.stamp = rospy.Time.now()
            msg.data = self.curr_depth.tostring()
            self.pub_image.publish(msg)
            imageio.imsave(f'img/depth_image_{self.depth_count}.png', image_np)
        
    def RGBCallback(self, data):
        if self.visualize:
            msg = msg_Image()
            msg.header.stamp = rospy.Time.now()
            msg.data = self.curr_rgb.tostring()
            self.pub_color_image.publish(msg)
            imageio.imsave(f'img/color_image_{self.rgb_count}.png', image_np)
        self.rgb_count +=1

    def get_camera_imgs(self):
        self.rgb_img = ros2numpy('rgb')
        self.depth_img = ros2numpy('depth')


def main():
    spot = Spot("CameraROS")
    mrn = CameraNode(spot)
    while not rospy.is_shutdown():
        mrn.publish_detection()

if __name__ == '__main__':
    main()