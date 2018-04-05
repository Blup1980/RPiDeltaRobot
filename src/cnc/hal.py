from cnc.path import *
from cnc.deltaRobot import *


class GHalException(Exception):
    """ Exceptions while processing gcode line.
    """
    pass


class HalFileExporter:
    def __init__(self):
        """ Initialize GPIO pins and machine itself.
        """
        self.robot = DeltaMechanics(L=DELTA_BIG_L, l=DELTA_SMALL_L, wb=DELTA_WB, up=DELTA_UP)
        logging.info("initialize hal")

    def check_valid_position(self, x, y, z):
        self.robot.update_from_new_tip_pos(np.array([x/1000, y/1000, z/1000+DELTA_Z_OFFSET]))
        return self.robot.valid

    # noinspection PyMethodMayBeStatic
    def spindle_control(self, percent):
        """ Spindle control implementation 0..100.
        :param percent: Spindle speed in percent.
        """
        logging.info("spindle control: {}%".format(percent))

    # noinspection PyMethodMayBeStatic
    def fan_control(self, on_off):
        """Cooling fan control.
        :param on_off: boolean value if fan is enabled.
        """
        if on_off:
            logging.info("Fan is on")
        else:
            logging.info("Fan is off")

    # noinspection PyMethodMayBeStatic
    def disable_steppers(self):
        """ Disable all steppers until any movement occurs.
        """
        logging.info("hal disable steppers")

    # noinspection PyMethodMayBeStatic
    def move(self, generator):
        """ Move head to specified position.
        :param generator: PulseGenerator object.
        """
        for tx, ty, tz, te in generator:
            self.robot.update_from_new_tip_pos(np.array([tx/1000, ty/1000, tz/1000+DELTA_Z_OFFSET]))
            if self.robot.valid:
                theta = self.robot.motor_angles()
                print("tip: {:f} {:f} {:f}".format(tx, ty, tz))
                self.print_rt("POS {:f} {:f} {:f}".format(MOTOR0_OFFSET_RAD - theta[0],
                                                          MOTOR1_OFFSET_RAD - theta[1],
                                                          MOTOR2_OFFSET_RAD - theta[2]))
            else:
                raise GHalException("Impossible delta geometry")

    # noinspection PyMethodMayBeStatic
    def join(self):
        """ Wait till motors work.
        """
        logging.info("hal join()")

    # noinspection PyMethodMayBeStatic
    def deinit(self):
        """ De-initialise.
        """
        logging.info("hal deinit()")

    @staticmethod
    def print_rt(strin):
        print("rt-cmd:{:s}".format(strin))
