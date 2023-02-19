#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def image_callback(image):
    print("Image received")
    global bridge
    try:
        cv_image = bridge.imgmsg_to_cv2(image,"bgr8")
    except CvBridgeError as e:
        print(e)
    cv2.imshow("Image", cv_image)
    cv2.waitKey(3)

def main():
    rospy.init_node("image_subscriber", anonymous=True)
    img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Error! Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
    