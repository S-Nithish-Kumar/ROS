#!/usr/bin/env python

import rospy
import cv2
import time
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def filter_color(rgb_img, yellowLower, yellowUpper):
    hsv_img = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv image", hsv_img)
    mask = cv2.inRange(hsv_img, yellowLower, yellowUpper)
    return mask

def getContours(binary_mask_img):
    contours, hierarchy = cv2.findContours(binary_mask_img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours

def draw_ball_contour(binary_mask_img, rgb_img, contours):
    black_img = np.zeros([binary_mask_img.shape[0],binary_mask_img.shape[1],3],'uint8')
    for c in contours:
        area = cv2.contourArea(c)
        ((x,y), radius) = cv2.minEnclosingCircle(c)
        if(area>3000):
            cv2.drawContours(rgb_img, [c], 0, (150, 250, 150), 2)
            cv2.drawContours(black_img, [c], 0, (150, 250, 150), 2)
            cx, cy = get_contour_center(c)
            cv2.circle(rgb_img, (cx, cy), (int)(radius),(0,0,255),1)
            cv2.circle(black_img, (cx, cy), (int)(radius),(0,0,255),1)
            cv2.circle(black_img, (cx, cy), 5,(150,150,255),-1)
    cv2.imshow("RGB image contours", rgb_img)
    cv2.imshow("Black image contours", black_img)

def get_contour_center(c):
    M = cv2.moments(c)
    if(M['m00']!=0):
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
    return cx, cy

def ball_detection_in_frame(img_frame):
    yellowLower = (30,100,50)
    yellowUpper = (60,255,255)
    rgb_img = img_frame
    binary_mask_img = filter_color(rgb_img, yellowLower, yellowUpper)
    contours = getContours(binary_mask_img)
    draw_ball_contour(binary_mask_img, rgb_img, contours)

def image_callback(image):
    print("Image received")
    global bridge
    try:
        cv_image = bridge.imgmsg_to_cv2(image,"bgr8")
    except CvBridgeError as e:
        print(e)
    ball_detection_in_frame(cv_image)
    #time.sleep(0.05)
    #cv2.imshow("Image", cv_image)
    cv2.waitKey(3)

def main():
    rospy.init_node("image_subscriber", anonymous=True)
    print("Welcome")
    img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Error! Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
    