from __future__ import division
import logging

from cnc.config import *
from cnc.coordinates import *
from math import floor

SECONDS_IN_MINUTE = 60.0


class PathGenerator:
    """ Stepper motors pulses generator.
        It generates time for each pulses for specified path as accelerated
        movement for specified velocity, then moves linearly and then braking
        with the same acceleration.
        Internally this class treat movement as uniform movement and then
        translate timings to accelerated movements. To do so, it base on
        formulas for distance of uniform movement and accelerated move.
            S = V * Ta = a * Tu^2 / 2
        where Ta - time for accelerated and Tu for uniform movement.
        Velocity will never be more then Vmax - maximum velocity of all axises.
        At the point of maximum velocity we change accelerated movement to
        uniform, so we can translate time for accelerated movement with this
        formula:
            Ta(Tu) = a * Tu^2 / Vmax / 2
        Now we need just to calculate how much time will accelerate and
        brake will take and recalculate time for them. Linear part will be as
        is. Since maximum velocity and acceleration is always the same, there
        is the ACCELERATION_FACTOR_PER_SEC variable.
        In the same way circular or other interpolation can be implemented
        based this class.
    """

    AUTO_VELOCITY_ADJUSTMENT = AUTO_VELOCITY_ADJUSTMENT

    def __init__(self, delta_mm, new_pos, velocity_mm_per_min):
        """ Create pulse generator for linear interpolation.
        class and implement interpolation function and related methods.
        All child have to call this method ( super().__init__() ).
        :param delta_mm: movement distance of each axis.
        :param velocity_mm_per_min: desired velocity.
        """
        self._iteration_x = 0
        self._iteration_y = 0
        self._iteration_z = 0
        self._iteration_e = 0
        self._acceleration_time_s = 0.0
        self._linear_time_s = 0.0
        self._2Vmax_per_a = 0.0
        self._delta = delta_mm
        self._start_pos = new_pos - delta_mm
        self._end_pos = new_pos

        distance_mm = abs(delta_mm)  # type: Coordinates
        # velocity of each axis
        distance_total_mm = distance_mm.length()
        self.max_velocity_mm_per_sec = self._adjust_velocity(distance_mm * (
            velocity_mm_per_min / SECONDS_IN_MINUTE / distance_total_mm))
        # acceleration time
        self.acceleration_time_s = (self.max_velocity_mm_per_sec.find_max() / TIP_MAX_ACCELERATION_MM_PER_S2)
        self.acceleration_time_s = floor(self.acceleration_time_s / REAL_TIME_DT) * REAL_TIME_DT

        # check if there is enough space to accelerate and brake, adjust time
        # S = a * t^2 / 2
        if TIP_MAX_ACCELERATION_MM_PER_S2 * self.acceleration_time_s ** 2 > distance_total_mm:
            self.acceleration_time_s = math.sqrt(distance_total_mm / TIP_MAX_ACCELERATION_MM_PER_S2)
            self.linear_time_s = 0.0
            # V = a * t -> V = 2 * S / t, take half of total distance for
            # acceleration and braking
            self.max_velocity_mm_per_sec = (distance_mm / self.acceleration_time_s)
        else:
            # calculate linear time
            acceleration_distance_mm = 1/2 * self.acceleration_time_s ** 2 * TIP_MAX_ACCELERATION_MM_PER_S2
            linear_distance_mm = distance_total_mm - 2*acceleration_distance_mm
            self.linear_time_s = (linear_distance_mm / self.max_velocity_mm_per_sec.length())

        self._total_time_steps = round(self.total_time_s() * REAL_TIME_DT)

    def _adjust_velocity(self, velocity_mm_sec):
        """ Automatically decrease velocity to all axises proportionally if
        velocity for one or more axises is more then maximum velocity for axis.
        :param velocity_mm_sec: input velocity.
        :return: adjusted(decreased if needed) velocity.
        """
        if not self.AUTO_VELOCITY_ADJUSTMENT:
            return velocity_mm_sec
        k = 1.0
        if velocity_mm_sec.x * SECONDS_IN_MINUTE > MAX_VELOCITY_MM_PER_MIN_X:
            k = min(k, MAX_VELOCITY_MM_PER_MIN_X
                    / velocity_mm_sec.x / SECONDS_IN_MINUTE)
        if velocity_mm_sec.y * SECONDS_IN_MINUTE > MAX_VELOCITY_MM_PER_MIN_Y:
            k = min(k, MAX_VELOCITY_MM_PER_MIN_Y
                    / velocity_mm_sec.y / SECONDS_IN_MINUTE)
        if velocity_mm_sec.z * SECONDS_IN_MINUTE > MAX_VELOCITY_MM_PER_MIN_Z:
            k = min(k, MAX_VELOCITY_MM_PER_MIN_Z
                    / velocity_mm_sec.z / SECONDS_IN_MINUTE)
        if velocity_mm_sec.e * SECONDS_IN_MINUTE > MAX_VELOCITY_MM_PER_MIN_E:
            k = min(k, MAX_VELOCITY_MM_PER_MIN_E
                    / velocity_mm_sec.e / SECONDS_IN_MINUTE)
        if k != 1.0:
            logging.warning("Out of speed, multiply velocity by {}".format(k))
        return velocity_mm_sec * k

    def _get_movement_parameters(self):
        """ Get parameters for interpolation. This method have to be
            reimplemented in parent classes and should calculate 3 parameters.
        :return: Tuple of three values:
                acceleration_time_s: time for accelerating and breaking motors
                                     during movement
                linear_time_s: time for uniform movement, it is total movement
                               time minus acceleration and braking time
                max_axis_velocity_mm_per_sec: maximum axis velocity of all
                                              axises during movement. Even if
                                              whole movement is accelerated,
                                              this value should be calculated
                                              as top velocity.
        """
        return (self.acceleration_time_s,
                self.linear_time_s,
                self.max_velocity_mm_per_sec)

    def __linear(self, i, velocity_mm_per_sec, end_position):
        """ Helper function for linear movement.
        """
        t = i * REAL_TIME_DT
        acceleration_steps = floor(self._acceleration_time_s / REAL_TIME_DT)
        linear_steps = self.linear_time_s / REAL_TIME_DT

        if acceleration_steps == 0:
            speed_increment = 0
        else:
            speed_increment = velocity_mm_per_sec / acceleration_steps

        acceleration = speed_increment / REAL_TIME_DT

        if i <= acceleration_steps:
            pos = 1/2 * acceleration * t**2
        elif i <= acceleration_steps + linear_steps:
            pos = 1/2 * acceleration * self._acceleration_time_s**2 \
                  + velocity_mm_per_sec * (t - self._acceleration_time_s)
        elif i < acceleration_steps + linear_steps + acceleration_steps:
            t_dec = t - (self._acceleration_time_s + self.linear_time_s)
            pos = 1/2 * acceleration * self._acceleration_time_s**2 \
                + velocity_mm_per_sec * self.linear_time_s \
                + velocity_mm_per_sec * t_dec \
                + 1/2 * -acceleration * t_dec**2
        elif i == (acceleration_steps + linear_steps + acceleration_steps):
            pos = end_position
        else:
            pos = None

        return pos

    def _interpolation_function(self, ix, iy, iz, ie):
        """ Get function for interpolation path. This function should returned
            values as it is uniform movement. There is only one trick, function
            must be expressed in terms of position, i.e. t = S / V for linear,
            where S - distance would be increment on motor minimum step.
        :param ix: number of pulse for X axis.
        :param iy: number of pulse for Y axis.
        :param iz: number of pulse for Z axis.
        :param ie: number of pulse for E axis.
        :return: Two tuples. First is tuple is directions for each axis,
                 positive means forward, negative means reverse. Second is
                 tuple of times for each axis in us or None if movement for
                 axis is finished.
        """
        dp_x = self.__linear(ix, self.max_velocity_mm_per_sec.x, self._delta.x)
        dp_y = self.__linear(iy, self.max_velocity_mm_per_sec.y, self._delta.y)
        dp_z = self.__linear(iz, self.max_velocity_mm_per_sec.z, self._delta.z)
        dp_e = self.__linear(ie, self.max_velocity_mm_per_sec.e, self._delta.e)
        return dp_x, dp_y, dp_z, dp_e

    def __iter__(self):
        """ Get iterator.
        :return: iterable object.
        """
        (self._acceleration_time_s, self._linear_time_s,
         max_axis_velocity_mm_per_sec) = self._get_movement_parameters()
        # helper variable
        self._2Vmax_per_a = (2.0 * max_axis_velocity_mm_per_sec.find_max()
                             / TIP_MAX_ACCELERATION_MM_PER_S2)
        self._iteration_x = 0
        self._iteration_y = 0
        self._iteration_z = 0
        self._iteration_e = 0
        logging.debug(', '.join("%s: %s" % i for i in vars(self).items()))
        return self

    def __next__(self):
        # for python3
        return self.next()

    def next(self):
        """ Iterate pulses.
        :return: Tuple of five values:
                    - values for all machine axises. For direction update,
                        positive values means forward movement, negative value
                        means reverse movement. For normal pulse, values are
                        represent time for the next pulse in microseconds.
                 This iteration strictly guarantees that next pulses time will
                 not be earlier in time then current. If there is no pulses
                 left StopIteration will be raised.
        """
        dp_x, dp_y, dp_z, dp_e = self._interpolation_function(self._iteration_x, self._iteration_y,
                                                              self._iteration_z, self._iteration_e)

        # check condition to stop
        if dp_x is None and dp_y is None and dp_z is None and dp_z is None:
            raise StopIteration

        if dp_x is not None:
            if self._delta.x < 0:
                dp_x = -dp_x
            self._iteration_x += 1
        if dp_y is not None:
            if self._delta.y < 0:
                dp_x = -dp_y
            self._iteration_y += 1
        if dp_z is not None:
            if self._delta.z < 0:
                dp_x = -dp_z
            self._iteration_z += 1
        if dp_z is not None:
            if self._delta.e < 0:
                dp_x = -dp_e
            self._iteration_e += 1

        return (self._start_pos.x + dp_x,
                self._start_pos.y + dp_y,
                self._start_pos.z + dp_z,
                self._start_pos.e + dp_e)

    def total_time_s(self):
        """ Get total time for movement.
        :return: time in seconds.
        """
        acceleration_time_s, linear_time_s, _ = self._get_movement_parameters()
        return acceleration_time_s * 2.0 + linear_time_s

    def delta(self):
        """ Get overall movement distance.
        :return: Movement distance for each axis in millimeters.
        """
        return self._delta

    def max_velocity(self):
        """ Get max velocity for each axis.
        :return: Vector with max velocity(in mm per min) for each axis.
        """
        _, _, v = self._get_movement_parameters()
        return v * SECONDS_IN_MINUTE
