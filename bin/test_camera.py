#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
import imutils

rospy.init_node('color_tracker')
rospy.loginfo('Tracker node started.')

# crea oggetto bridge e il publisher per le coordinate
bridge = CvBridge()


# funzione callback
def callback(data):
    try:
        """ Convert the raw image to OpenCV format """
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

        cv2.imshow('hsv', cv_image)
        # cv2.imshow('mask', mask)
        cv2.waitKey(3)

    except CvBridgeError, e:
        print e


def main():
    # Sottoscreive il raw camera image topic e pubblica il RoI
    camera_sub = rospy.Subscriber("/usb_cam/image_raw", Image, callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        # TODO: migliorare uscita dal programma
        print 'Process shutting down'
        cv2.destroyAllWindows()


# Main routine
if __name__ == '__main__':
    main()

