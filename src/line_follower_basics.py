#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

"""we create a class with the line follower basic functionalities
#such as 1. collecting image
# 2. creating the cv bridge object and launching the image window
"""
class LineFollower(object):

    #first some objects are declared which will be used throughout the class
    def __init__(self):

        self.bridge_object = CvBridge()#creating the CVBridge object for converting images
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)#adding the subscriber to the topic of the camera imgs
#this is the callback function for the camera images! objects from this function are used later
    def camera_callback(self,data):

        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            #convert the image data contained in an ImageSensorData into a format that OpenCV understands
        except CvBridgeError as e:
            print(e)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)

#we initialize this node as the linefollowing node
def main():
    line_follower_object = LineFollower()
    rospy.init_node('line_following_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()