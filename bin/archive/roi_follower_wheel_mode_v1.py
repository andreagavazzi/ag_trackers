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
print 'Connecting to the first available port:', port

dxl_io = pypot.dynamixel.DxlIO(port)
ids = dxl_io.scan([1, 2])
print 'Connecting to Dinamixel id:', ids
print 'Ready to go!'


# Rest position
def zero():
    dxl_io.disable_torque([1, 2])
    dxl_io.set_joint_mode([1, 2])
    dxl_io.enable_torque([1, 2])
    dxl_io.set_moving_speed({1: 250, 2: 250})
    dxl_io.set_goal_position({1: 0, 2: 0})
    time.sleep(1)
    dxl_io.disable_torque([1, 2])
    dxl_io.set_wheel_mode([1, 2])
    dxl_io.enable_torque([1, 2])
    dxl_io.set_moving_speed({1: 0, 2: 0})


speed = 120  # base = 120
tolerance = 80
# limiti dei dynamixel
pan_min = -75
pan_max = 75
tilt_min = -45
tilt_max = 30
rate = rospy.Rate(0.05)


def callback(data):
    roi = data     # riceve le info sui dynamixel
    roi_x_centre = roi.width/2 + roi.x_offset       # centro del roi x
    roi_y_centre = roi.height/2 + roi.y_offset      # centro del roi y

    current_pos = dxl_io.get_present_position((1, 2))
    current_pos_x = current_pos[0]
    current_pos_y = current_pos[1]


# TODO: se il roi non cambia allora torna a zero dopo tot secondi
# TODO: zona tolleranza proporzionale alla larghezza del roi

    if 400 - tolerance <= roi_x_centre <= 400 + tolerance:      # il centro immagine
        dxl_io.set_moving_speed({1: 0})
        rospy.loginfo("01")
    elif roi_x_centre > 400 + tolerance and current_pos_x > pan_min:     # current pos deve stare nel limite del servo
        dxl_io.set_moving_speed({1: -1*speed})
        rospy.loginfo("02")
    elif roi_x_centre < 400 - tolerance and current_pos_x < pan_max:      # current pos deve stare nel limite del servo
        dxl_io.set_moving_speed({1: speed})
        rospy.loginfo("03")
        rospy.loginfo(current_pos_x)
    else:
        dxl_io.set_moving_speed({1: 0})
        rospy.loginfo("04")


    if 300 - tolerance <= roi_y_centre <= 300 + tolerance:      # il centro immagine
        dxl_io.set_moving_speed({2: 0})
    elif roi_y_centre > 300 + tolerance and current_pos_y < tilt_max:      # current pos deve stare nel limite del servo
        dxl_io.set_moving_speed({2: speed})
    elif roi_y_centre < 300 - tolerance and current_pos_y > tilt_min:    # current pos deve stare nel limite del servo
        dxl_io.set_moving_speed({2: -1*speed})
    else:
        dxl_io.set_moving_speed({2: 0})


def main():
    zero()
    while not rospy.is_shutdown():
        rospy.Subscriber("/roi", RegionOfInterest, callback, queue_size=1)
        dxl_io.set_moving_speed({1: 0, 2: 0})
        rospy.loginfo("Out of sight")
        rate.sleep()
    rospy.on_shutdown(shutdown)


def shutdown():
    print " Shutting down..."
    zero()
    dxl_io.close()
    print "Terminated"


# Main routine
if __name__ == '__main__':
    main()
