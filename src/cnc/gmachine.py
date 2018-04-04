from __future__ import division

import cnc.logging_config as logging_config
from cnc.path import *
from cnc.coordinates import *
from cnc.enums import *
import cnc.hal as hal
import time


class GMachineException(Exception):
    """ Exceptions while processing gcode line.
    """
    pass


class GMachine(object):
    """ Main object which control and keep state of whole machine: steppers,
        spindle, extruder etc
    """

    def __init__(self):
        """ Initialization.
        """
        self._position = Coordinates(0.0, 0.0, 0.0, 0.0)
        # init variables
        self._velocity = 0
        self._spindle_rpm = 0
        self._local = None
        self._convertCoordinates = 0
        self.reset()
        hal.init()

    def release(self):
        """ Free all resources.
        """
        self._spindle(0)
        hal.deinit()

    def reset(self):
        """ Reinitialize all program configurable thing.
        """
        self._velocity = min(MAX_VELOCITY_MM_PER_MIN_X,
                             MAX_VELOCITY_MM_PER_MIN_Y,
                             MAX_VELOCITY_MM_PER_MIN_Z,
                             MAX_VELOCITY_MM_PER_MIN_E)
        self._spindle_rpm = 1000
        self._local = Coordinates(0.0, 0.0, 0.0, 0.0)
        self._convertCoordinates = 1.0

    # noinspection PyMethodMayBeStatic
    def _spindle(self, spindle_speed):
        hal.join()
        hal.spindle_control(100.0 * spindle_speed / SPINDLE_MAX_RPM)

    # noinspection PyMethodMayBeStatic
    def __check_velocity(self, max_velocity):
        if max_velocity.x > MAX_VELOCITY_MM_PER_MIN_X \
                or max_velocity.y > MAX_VELOCITY_MM_PER_MIN_Y \
                or max_velocity.z > MAX_VELOCITY_MM_PER_MIN_Z \
                or max_velocity.e > MAX_VELOCITY_MM_PER_MIN_E:
            raise GMachineException("out of maximum speed")

    def _move_linear(self, new_pos, velocity):
        delta = new_pos - self._position
        if delta.is_zero():
            return
        if not new_pos.is_in_aabb(Coordinates(0.0, 0.0, 0.0, 0.0),
                                  Coordinates(TABLE_SIZE_X_MM, TABLE_SIZE_Y_MM,
                                              TABLE_SIZE_Z_MM, 0)):
            raise GMachineException("out of effective area")

        logging.info("Moving linearly to{}".format(new_pos))
        gen = PathGenerator(delta, new_pos, velocity)
        self.__check_velocity(gen.max_velocity())
        hal.move(gen)
        # save position
        self._position = new_pos

    @staticmethod
    def __quarter(pa, pb):
        if pa >= 0 and pb >= 0:
            return 1
        if pa < 0 and pb >= 0:
            return 2
        if pa < 0 and pb < 0:
            return 3
        if pa >= 0 and pb < 0:
            return 4

    def safe_zero(self, x=True, y=True, z=True):
        """ Move head to zero position safely.
        :param x: boolean, move X axis to zero
        :param y: boolean, move Y axis to zero
        :param z: boolean, move Z axis to zero
        """
        if x and not y:
            self._move_linear(Coordinates(0,
                                          self._position.y,
                                          self._position.z,
                                          self._position.e),
                              MAX_VELOCITY_MM_PER_MIN_X)
        elif y and not x:
            self._move_linear(Coordinates(self._position.x,
                                          0,
                                          self._position.z,
                                          self._position.e),
                              MAX_VELOCITY_MM_PER_MIN_X)
        elif x and y:
            self._move_linear(Coordinates(0,
                                          0,
                                          0,
                                          self._position.e),
                              min(MAX_VELOCITY_MM_PER_MIN_X,
                                  MAX_VELOCITY_MM_PER_MIN_Y))
        if z:
            self._move_linear(Coordinates(self._position.x,
                                          self._position.y,
                                          0, self._position.e),
                              MAX_VELOCITY_MM_PER_MIN_Z)

    def position(self):
        """ Return current machine position (after the latest command)
            Note that hal might still be moving motors and in this case
            function will block until motors stops.
            This function for tests only.
            :return current position.
        """
        hal.join()
        return self._position

    def do_command(self, gcode):
        """ Perform action.
        :param gcode: GCode object which represent one gcode line
        :return String if any answer require, None otherwise.
        """
        if gcode is None:
            return None
        answer = None
        logging.debug("got command " + str(gcode.params))
        # read command
        c = gcode.command()
        if c is None and gcode.has_coordinates():
            c = 'G1'
        # read parameters

        coord = gcode.coordinates(self._position - self._local,
                                  self._convertCoordinates)
        coord = coord + self._local
        delta = coord - self._position

        velocity = gcode.get('F', self._velocity)
        # check parameters
        if velocity < MIN_VELOCITY_MM_PER_MIN:
            raise GMachineException("feed speed too low")
        # select command and run it
        if c == 'G1':  # linear interpolation
            self._move_linear(coord, velocity)
        elif c == 'G4':  # delay in s
            if not gcode.has('P'):
                raise GMachineException("P is not specified")
            pause = gcode.get('P', 0)
            if pause < 0:
                raise GMachineException("bad delay")
            hal.join()
            time.sleep(pause)
        elif c == 'G20':  # switch to inches
            self._convertCoordinates = 25.4
        elif c == 'G21':  # switch to mm
            self._convertCoordinates = 1.0
        elif c == 'G28':  # home
            axises = gcode.has('X'), gcode.has('Y'), gcode.has('Z')
            if axises == (False, False, False):
                axises = True, True, True
            self.safe_zero(*axises)
            hal.join()
            if not hal.calibrate(*axises):
                raise GMachineException("failed to calibrate")
        elif c == 'G53':  # switch to machine coords
            self._local = Coordinates(0.0, 0.0, 0.0, 0.0)
        elif c == 'G90':  # switch to absolute coords
            pass
        elif c == 'G91':  # switch to relative coords
            raise GMachineException("Not supported")
        elif c == 'G92':  # switch to local coords
            if gcode.has_coordinates():
                self._local = self._position - gcode.coordinates(
                    Coordinates(self._position.x - self._local.x,
                                self._position.y - self._local.y,
                                self._position.z - self._local.z,
                                self._position.e - self._local.e),
                    self._convertCoordinates)
            else:
                self._local = self._position
        elif c == 'M3':  # spindle on
            spindle_rpm = gcode.get('S', self._spindle_rpm)
            if spindle_rpm < 0 or spindle_rpm > SPINDLE_MAX_RPM:
                raise GMachineException("bad spindle speed")
            self._spindle(spindle_rpm)
            self._spindle_rpm = spindle_rpm
        elif c == 'M5':  # spindle off
            self._spindle(0)
        elif c == 'M2' or c == 'M30':  # program finish, reset everything.
            self.reset()
        elif c == 'M84':  # disable motors
            hal.disable_steppers()
        elif c == 'M111':  # enable debug
            logging_config.debug_enable()
        elif c == 'M114':  # get current position
            hal.join()
            p = self.position()
            answer = "X:{} Y:{} Z:{} E:{}".format(p.x, p.y, p.z, p.e)
        elif c is None:  # command not specified(ie just F was passed)
            pass
        else:
            raise GMachineException("unknown command")
        # save parameters on success
        self._velocity = velocity
        logging.debug("position {}".format(self._position))
        return answer
