# !/usr/bin/env python

# Sottoscrive un topic /roi con messaggio RegionOfInterest e muove un dispositivo Pan&Tilt

import rospy
from sensor_msgs.msg import RegionOfInterest
import time
import pypot.dynamixel

rospy.init_node('drive_head')

ports = pypot.dynamixel.get_available_ports()

if not ports:
    raise IOError('no port found!')

port = ports[0]
rospy.loginfo('Connecting to the first available port: ' + str(port))

dxl_io = pypot.dynamixel.DxlIO(port)
ids = dxl_io.scan([1, 2])
rospy.loginfo('Connecting to Dinamixel id: ' + str(ids))
rospy.loginfo('drive_head node started!')


# Rest position - joint mode
def zero():
    dxl_io.disable_torque([1, 2])
    dxl_io.set_joint_mode([1, 2])
    dxl_io.enable_torque([1, 2])
    dxl_io.set_moving_speed({1: 250, 2: 250})
    dxl_io.set_goal_position({1: 0, 2: 0})
    time.sleep(1)
    stop(1)
    stop(2)


speed = 120  # base = 120
tolerance = 80
# limiti dei dynamixel
pan_min = -75
pan_max = 75
tilt_min = -45
tilt_max = 30
rate = rospy.Rate(0.05)


def stop(s):
    dxl_io.set_moving_speed({s: 0, })


def callback(data):
    roi = data     # riceve le info sui dynamixel
    roi_x_centre = roi.width/2 + roi.x_offset       # centro del roi x
    roi_y_centre = roi.height/2 + roi.y_offset      # centro del roi y

    current_pos = dxl_io.get_present_position((1, 2))
    current_pos_x = current_pos[0]
    current_pos_y = current_pos[1]


def main():
    zero()
    while not rospy.is_shutdown():
        rospy.Subscriber("/roi", RegionOfInterest, callback, queue_size=1)
        rospy.spin()

    rospy.on_shutdown(shutdown)


def shutdown():
    zero()
    dxl_io.close()
    rospy.loginfo("drive_head node terminated by user!")


# Main routine
if __name__ == '__main__':
    main()
