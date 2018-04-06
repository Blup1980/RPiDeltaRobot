from unittest import TestCase
from cnc.path import *
from cnc.coordinates import Coordinates
from cnc.config import *
import numpy as np


class TestPathGenerator(TestCase):

    def test_max_speed_x(self):
        max_possible_speed = MAX_VELOCITY_MM_PER_MIN_X
        delta_mm = Coordinates(60, 0, 0, 0)
        for speed in (1, int(math.floor(max_possible_speed))):
            self._speed_check(speed, delta_mm)

    def test_max_speed_y(self):
        max_possible_speed = MAX_VELOCITY_MM_PER_MIN_Y
        delta_mm = Coordinates(0, 60, 0, 0)
        for speed in (1, int(math.floor(max_possible_speed))):
            self._speed_check(speed, delta_mm)

    def test_max_speed_z(self):
        max_possible_speed = MAX_VELOCITY_MM_PER_MIN_Z
        delta_mm = Coordinates(0, 0, 60, 0)
        for speed in (1, int(math.floor(max_possible_speed))):
            self._speed_check(speed, delta_mm)

    def test_end_position(self):
        targets = [Coordinates(x, x, x, 0) for x in range(1, 101, 20)]
        too_fast_speed = math.ceil(max(MAX_VELOCITY_MM_PER_MIN_X,
                                       MAX_VELOCITY_MM_PER_MIN_Y,
                                       MAX_VELOCITY_MM_PER_MIN_Z)
                                   * 1.1)
        velocities = [1, too_fast_speed]

        for target_pos in targets:
            delta_mm = target_pos
            for velocity_mm_per_min in velocities:
                self._pos_check(target_pos, delta_mm, velocity_mm_per_min)

    def test_position_overshoot_x(self):
        targets = [Coordinates(x, 0, 0, 0) for x in [60]]
        too_fast_speed = MAX_VELOCITY_MM_PER_MIN_X * 1.1
        velocities = [too_fast_speed]
        for target_pos in targets:
            delta_mm = target_pos
            for velocity_mm_per_min in velocities:
                self._overshoot_check(target_pos, delta_mm, velocity_mm_per_min)

    def _overshoot_check(self, target_pos, delta_mm, velocity_mm_per_min):
        tolerance_digit = 1
        dut = PathGenerator(delta_mm, target_pos, velocity_mm_per_min)
        max_pos = Coordinates(0, 0, 0, 0)
        for px, py, pz, pe in dut:
            p = Coordinates(px, py, pz, 0)
            max_pos.x = max(max_pos.x, p.x)
            max_pos.y = max(max_pos.y, p.y)
            max_pos.z = max(max_pos.z, p.z)

        distance_error = max_pos - target_pos
        self.assertAlmostEqual(distance_error.length(), 0.0, tolerance_digit)

    def _pos_check(self, target_pos, delta_mm, velocity_mm_per_min):
        tolerance_digit = 1
        dut = PathGenerator(delta_mm, target_pos, velocity_mm_per_min)
        last_pos = Coordinates(0, 0, 0, 0)
        for px, py, pz, pe in dut:
            last_pos = Coordinates(px, py, pz, 0)

        distance_error = last_pos - target_pos
        self.assertAlmostEqual(distance_error.length(), 0.0, tolerance_digit)

    def _speed_check(self, velocity_mm_per_min, delta_mm):
        #print("testing {:.2f} mm/min".format(velocity_mm_per_min))
        new_pos = delta_mm
        dut = PathGenerator(delta_mm, new_pos, velocity_mm_per_min)
        old_pos = Coordinates(0, 0, 0, 0)
        max_speed = 0
        for px, py, pz, pe in dut:
            p = Coordinates(px, py, pz, 0)
            delta = p - old_pos
            old_pos = p
            v = delta.length() / REAL_TIME_DT
            max_speed = max(max_speed, v)

        self.assertNotEqual(dut.linear_time_s, 0.0,
                            "Not enough space to reach the max speed ({:.2f}mm/min) given that acceleration"
                            .format(velocity_mm_per_min))
        self.assertAlmostEqual(max_speed, velocity_mm_per_min / SECONDS_IN_MINUTE, 1,
                               "Effective max speed not correct when trying {:f} mm/min".format(velocity_mm_per_min))
