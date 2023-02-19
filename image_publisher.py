#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def main():
    global bridge
    pub = rospy.Publisher('/usb_cam/image_raw', Image, queue_size = 10)
    rospy.init_node('image_publisher', anonymous=True)

    video_capture = cv2.VideoCapture(0)

    while not rospy.is_shutdown():
        video_status, frame = video_capture.read()
        try:
            cv_image = bridge.cv2_to_imgmsg(frame, "bgr8")
            pub.publish(cv_image)
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    main()
