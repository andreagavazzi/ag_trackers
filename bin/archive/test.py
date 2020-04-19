
import itertools
import numpy
import time

import pypot.dynamixel as dyna

AMP = 30
FREQ = 0.5

if __name__ == '__main__':

    dxl_io = dyna.DxlIO('/dev/ttyUSB0')
    print('Connected!')

    found_ids = dxl_io.scan([1, 2])
    print('Found ids:', found_ids)

    if len(found_ids) < 2:
        raise IOError('You should connect at least two motors on the bus for this test.')

    ids = found_ids[:1]

    dxl_io.enable_torque(ids)

    speed = dict(zip(ids, itertools.repeat(200)))
    dxl_io.set_moving_speed(speed)

    pos = dict(zip(ids, itertools.repeat(0)))
    dxl_io.set_goal_position(pos)

    t0 = time.time()
    while True:
        t = time.time()
        if (t - t0) > 10:
            break


        pos = AMP * numpy.sin(2 * numpy.pi * FREQ * t)
        dxl_io.set_goal_position(dict(zip(ids, itertools.repeat(pos))))

        time.sleep(0.02)

