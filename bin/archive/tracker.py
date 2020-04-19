#!/usr/bin/env python

import roslib
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import imutils

rospy.init_node('color_tracker', anonymous=True)

# crea oggetto bridge e il publisher per le coordinate
bridge = CvBridge()
coord_pub = rospy.Publisher('coords', String, queue_size=10)

# colore HSV da tracciare
colorLower = (73, 225, 73)
colorUpper = (133, 285, 193)


# funzione callback
def callback(data):
    try:
        """ Convert the raw image to OpenCV format """
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.waitKey(3)
        frame = imutils.resize(cv_image, width=600)
        blurred = cv2.GaussianBlur(cv_image, (13, 13), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, colorLower, colorUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None

        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # Procede solo se la dimensione del raggio e rilevante
            if radius > 10:
                # Crea il cerchio e aggiorna le coordinate nel frame
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 0, 255), 2)
                cv2.circle(frame, center, 3, (0, 0, 255))

                cv2.rectangle(frame, (int(x-radius), int(y-radius)), (int(x+radius), int(y+radius)), (0, 0, 255))
                coord = str(int(x)) + ',' + str(int(y))
                cv2.putText(frame, coord, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

                # Pubblica le coordinate
                coord_pub.publish(coord)

        # Mostra il video
        cv2.imshow('Camera', frame)
        # Mostra la maschera
        cv2.imshow('Mask', mask)
        cv2.waitKey(3)

    except CvBridgeError, e:
        print e


def main():
    """ Subscribe to the raw camera image topic and publish the coordinates of the center"""
    camera_sub = rospy.Subscriber("/camera/color/image_raw", Image, callback)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Shutting down vision node.')
        cv2.destroyAllWindows()


# Main routine
if __name__ == '__main__':
    main()

