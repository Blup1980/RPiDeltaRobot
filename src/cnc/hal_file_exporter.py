from __future__ import division
import time

from cnc.path import *
from cnc.config import *

""" This is virtual device class which is very useful for debugging.
    It checks PulseGenerator with some tests.
"""


def init():
    """ Initialize GPIO pins and machine itself.
    """
    logging.info("initialize hal")


def spindle_control(percent):
    """ Spindle control implementation 0..100.
    :param percent: Spindle speed in percent.
    """
    logging.info("spindle control: {}%".format(percent))


def fan_control(on_off):
    """Cooling fan control.
    :param on_off: boolean value if fan is enabled.
    """
    if on_off:
        logging.info("Fan is on")
    else:
        logging.info("Fan is off")


def disable_steppers():
    """ Disable all steppers until any movement occurs.
    """
    logging.info("hal disable steppers")


# noinspection PyUnusedLocal
def move(generator):
    """ Move head to specified position.
    :param generator: PulseGenerator object.
    """
    delta = generator.delta()
    ix = iy = iz = ie = 0
    lx, ly, lz, le = None, None, None, None
    dx, dy, dz, de = 0, 0, 0, 0
    mx, my, mz, me = 0, 0, 0, 0
    cx, cy, cz, ce = 0, 0, 0, 0
    direction_x, direction_y, direction_z, direction_e = 1, 1, 1, 1
    st = time.time()
    direction_found = False
    for tx, ty, tz, te in generator:
        print(tx, ty, tz, te)

    pt = time.time()
    logging.debug("Moved {}, {}, {}, {} iterations".format(ix, iy, iz, ie))
    logging.info("prepared in " + str(round(pt - st, 2)) + "s, estimated "
                 + str(round(generator.total_time_s(), 2)) + "s")


def join():
    """ Wait till motors work.
    """
    logging.info("hal join()")


def deinit():
    """ De-initialise.
    """
    logging.info("hal deinit()")

