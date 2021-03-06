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
roi_pub = rospy.Publisher('roi', RegionOfInterest, queue_size=1)

# Inizializza il messaggio
roi = RegionOfInterest()

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
        frame = cv_image
        blurred = cv2.GaussianBlur(cv_image, (13, 13), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, colorLower, colorUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        # center = None

        if len(cnts) > 0:
            # Trova il contorno largo nella maschera e lo utilizza per calcora il minEnclosingCircle
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            # Il codice commentato serve a disegnare un cerchio invece del box
            # m = cv2.moments(c)
            # center = (int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"]))

            # Procede solo se ho una dimensione del raggio rilevante
            if radius > 10:
                # Crea il cerchio e aggiorna le coordinate nel frame
                # cv2.circle(frame, (int(x), int(y)), int(radius), (0, 0, 255), 2)
                # cv2.circle(frame, center, 3, (0, 0, 255))
                cv2.rectangle(frame, (int(x - radius), int(y - radius)), (int(x + radius), int(y + radius)),
                              (0, 0, 255))

                coord = str(int(x)) + ',' + str(int(y))
                cv2.putText(frame, coord, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

                # Il msg RoI necessita valori positivi, va gestito il fuori asse negativo
                if x >= radius:         # la situazione normale
                    roi.x_offset = int(x - radius)
                    roi.width = int(2 * radius)
                else:               # quando il rettagolo roi entra nell'asse negativa fisso a 0 e dimezzo il
                    roi.x_offset = 0
                    roi.width = int(2 * radius + (x - radius))         # x-radius ha valore negativo
                if y >= radius:         # la situazione normale
                    roi.y_offset = int(y - radius)
                    roi.height = int(2 * radius)
                else:
                    roi.y_offset = 0
                    roi.height = int(2 * radius + (y-radius))       # y-radius ha valore negativo

                roi_pub.publish(roi)

        else:
            roi.x_offset = roi.y_offset = 0
            roi.width = roi.height = 0
            roi_pub.publish(roi)

        # Mostra il video
        cv2.imshow('Camera', frame)
        # Mostra la maschera in una seconda finestra per debug
        # cv2.imshow('hsv', hsv)
        # cv2.waitKey(3)

    except CvBridgeError, e:
        print e


def main():
    # Sottoscreive il raw camera image topic e pubblica il RoI
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
