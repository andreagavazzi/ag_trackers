# !/usr/bin/env python

# Sottoscrive un topic /roi con messaggio RegionOfInterest e muove un dispositivo Pan&Tilt

import rospy
from sensor_msgs.msg import RegionOfInterest
import time

rospy.init_node('test_node')
rate = rospy.Rate(10)


def callback(data):
    print 'inside callback'
    roi = data
    a = roi.x_offset
    b = roi.y_offset
    print a, b, ': ', time.ctime()


def shutdown():
    print 'Chiusura'


def main():
    while not rospy.is_shutdown():
        rospy.Subscriber("/roi", RegionOfInterest, callback, queue_size=1)
        rate.sleep()
        print 'outside callback'


# Main routine
if __name__ == '__main__':
    main()

