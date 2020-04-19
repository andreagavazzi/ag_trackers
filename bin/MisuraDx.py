# !/usr/bin/env python

# Sottoscrive un topic /roi con messaggio RegionOfInterest e muove un dispositivo Pan&Tilt

import pypot.dynamixel

ports = pypot.dynamixel.get_available_ports()

if not ports:
    raise IOError('no port found!')

port = ports[0]
print 'Connecting to the first available port:', port

dxl_io = pypot.dynamixel.DxlIO(port)
ids = dxl_io.scan([1, 2])
print 'Connecting to Dinamixel id:', ids
print 'Ready to go!'


def main():
    current_pos_x = dxl_io.get_present_position((1, 2))
    # current_pos_y = dxl_io.get_present_position((2,))
    print current_pos_x[0], current_pos_x[1]


if __name__ == '__main__':
    while True:
        main()


