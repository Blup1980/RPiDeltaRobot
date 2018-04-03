import time

from cnc.pulses import *
from cnc.config import *

US_IN_SECONDS = 1000000


def init():
    """ Initialize GPIO pins and machine itself.
    """


def spindle_control(percent):
    """ Spindle control implementation.
    :param percent: spindle speed in percent 0..100. If 0, stop the spindle.
    """
    logging.info("spindle control: {}%".format(percent))
    if percent > 0:
        pass
    else:
        pass


def fan_control(on_off):
    """
    Cooling fan control.
    :param on_off: boolean value if fan is enabled.
    """
    if on_off:
        logging.info("Fan is on")
        pass
    else:
        logging.info("Fan is off")
        pass


def disable_steppers():
    """ Disable all steppers until any movement occurs.
    """
    logging.info("disable steppers")
    pass


def calibrate(x, y, z):
    """ Move head to home position till end stop switch will be triggered.
    Do not return till all procedures are completed.
    :param x: boolean, True to calibrate X axis.
    :param y: boolean, True to calibrate Y axis.
    :param z: boolean, True to calibrate Z axis.
    :return: boolean, True if all specified end stops were triggered.
    """
    return True


def move(generator):
    """ Move head to specified position
    :param generator: PulseGenerator object.
    """
    prev = 0
    is_ran = False
    instant = INSTANT_RUN
    st = time.time()
    current_cb = 0
    k = 0
    k0 = 0
    for direction, tx, ty, tz, te in generator:
        pass
        # print([direction, tx, ty, tz, te])
    pt = time.time()

    logging.info("prepared in " + str(round(pt - st, 2)) + "s, estimated in "
                 + str(round(generator.total_time_s(), 2)) + "s")


def join():
    """ Wait till motors work.
    """
    logging.info("hal join()")
    time.sleep(0.01)


def deinit():
    """ De-initialize hardware.
    """
    join()
    pass


def watchdog_feed():
    """ Feed hardware watchdog.
    """
    pass
