#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node('dice_tracker')
rospy.loginfo('Tracker node started.')

# crea oggetto bridge e il publisher per le coordinate
bridge = CvBridge()
roi_pub = rospy.Publisher('roi', RegionOfInterest, queue_size=1)

# Inizializza il messaggio
roi = RegionOfInterest()

# Impostazioni camera
min_threshold = 10
max_threshold = 200
min_area = 100
min_circularity = .3
min_inertia_ratio = .5


# funzione callback
def callback(data):
    try:
        """ Convert the raw image to OpenCV format """
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.waitKey(3)
        # cv_image.set(cv2.CAP_PROP_EXPOSURE, 0.25)
        # frame = imutils.resize(cv_image, width=600)
        frame = cv_image
        # cv2.imshow('Camera', frame)
    # !!!!!!!!! Codice

        # if counter_p >= 90000:  # set maximum sizes for variables and lists to save memory.
        #     counter_p = 0
        #     readings_p = [0, 0]
        #     display_p = [0, 0]

        #
        params = cv2.SimpleBlobDetector_Params()  # declare filter parameters.
        params.filterByArea = True
        params.filterByCircularity = True
        params.filterByInertia = True
        params.minThreshold = min_threshold
        params.maxThreshold = max_threshold
        params.minArea = min_area
        params.minCircularity = min_circularity
        params.minInertiaRatio = min_inertia_ratio
        #
        detector = cv2.SimpleBlobDetector_create(params)  # create a blob detector object.
        #
        keypoints = detector.detect(cv_image)
        #
        # # here we draw keypoints on the frame.
        im_with_keypoints = cv2.drawKeypoints(cv_image, keypoints, np.array([]), (0, 0, 255),
                                              cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        #
        cv2.imshow("image KP", im_with_keypoints)  # display the frame with keypoints added.
        #
        reading = len(keypoints)  # 'reading' counts the number of keypoints (pips).
        #
        # print reading


# !!!!!! Fine codice





        # if len(cnts) > 0:
        #     # Trova il contorno largo nella maschera e lo utilizza per calcora il minEnclosingCircle
        #     c = max(cnts, key=cv2.contourArea)
        #     ((x, y), radius) = cv2.minEnclosingCircle(c)
        #     # Il codice commentato serve a disegnare un cerchio invece del box
        #     # m = cv2.moments(c)
        #     # center = (int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"]))
        #
        #     # Procede solo se ho una dimensione del raggio rilevante
        #     if radius > 10:
        #         # Crea il cerchio e aggiorna le coordinate nel frame
        #         # cv2.circle(frame, (int(x), int(y)), int(radius), (0, 0, 255), 2)
        #         # cv2.circle(frame, center, 3, (0, 0, 255))
        #         cv2.rectangle(frame, (int(x - radius), int(y - radius)), (int(x + radius), int(y + radius)),
        #                       (0, 0, 255))
        #
        #         coord = str(int(x)) + ',' + str(int(y))
        #         cv2.putText(frame, coord, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        #
        #         # Il msg RoI necessita valori positivi, va gestito il fuori asse negativo
        #         if x >= radius:         # la situazione normale
        #             roi.x_offset = int(x - radius)
        #             roi.width = int(2 * radius)
        #         else:               # quando il rettagolo roi entra nell'asse negativa fisso a 0 e dimezzo il
        #             roi.x_offset = 0
        #             roi.width = int(2 * radius + (x - radius))         # x-radius ha valore negativo
        #         if y >= radius:         # la situazione normale
        #             roi.y_offset = int(y - radius)
        #             roi.height = int(2 * radius)
        #         else:
        #             roi.y_offset = 0
        #             roi.height = int(2 * radius + (y-radius))       # y-radius ha valore negativo
        #
        #         roi_pub.publish(roi)
        #
        # else:
        #     roi.x_offset = roi.y_offset = 0
        #     roi.width = roi.height = 0
        #     roi_pub.publish(roi)

        # Mostra il video
        # cv2.imshow('Camera', frame)
        # Mostra la maschera in una seconda finestra per debug
        # cv2.imshow('hsv', hsv)
        # cv2.waitKey(3)

    except CvBridgeError, e:
        print e


def main():
    # Sottoscrive il raw camera image topic e pubblica il RoI
    while not rospy.is_shutdown():
        camera_sub = rospy.Subscriber("/camera/color/image_raw", Image, callback)
        rospy.spin()
    rospy.on_shutdown(shutdown)


def shutdown():
    cv2.destroyAllWindows()
    rospy.loginfo('Tracker node terminated.')


# Main routine
if __name__ == '__main__':
    main()
