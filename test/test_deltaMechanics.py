import unittest
from cnc import deltaRobot
import numpy as np


class TestDeltaMechanics(unittest.TestCase):
    def test_creating(self):
        dut = deltaRobot.DeltaMechanics(L=0.524, l=1.244, wb=0.164, up=0.044)
        self.assertEqual(dut.valid, False)

    def test_possible_position(self):
        dut = deltaRobot.DeltaMechanics(L=0.524, l=1.244, wb=0.164, up=0.044)
        """example of computation given from the pdf"""
        dut.update_from_new_tip_pos(np.array([0, 0, -0.9]))
        theta = dut.motor_angles()
        self.assertAlmostEqual(theta[0], np.deg2rad(-20.5), 2)
        self.assertAlmostEqual(theta[1], np.deg2rad(-20.5), 2)
        self.assertAlmostEqual(theta[2], np.deg2rad(-20.5), 2)
        self.assertEqual(dut.valid, True)

        dut.update_from_new_tip_pos(np.array([0.3, 0.5, -1.1]))
        theta = dut.motor_angles()
        self.assertAlmostEqual(theta[0], np.deg2rad(47.5), 2)
        self.assertAlmostEqual(theta[1], np.deg2rad(-11.6), 2)
        self.assertAlmostEqual(theta[2], np.deg2rad(21.4), 2)
        self.assertEqual(dut.valid, True)

    def test_impossible_position(self):
        dut = deltaRobot.DeltaMechanics(L=0.524, l=1.244, wb=0.164, up=0.044)
        dut.update_from_new_tip_pos(np.array([0, 0, 10]))
        self.assertEqual(dut.valid, False)

    def test_wrong_size_vector_speed(self):
        dut = deltaRobot.DeltaMechanics(L=0.524, l=1.244, wb=0.164, up=0.044)
        dut.update_from_new_tip_pos(np.array([0.3, 0.5, -1.1]))
        tip_speed = np.array([1, 1, 1])
        self.assertRaises(ValueError, dut.update_motor_speed_from_tip_speed, tip_speed)

    def test_speed_computation(self):
        """ Speed is computed using numerical approximation using the v = deltaPos/deltaTime formula and
        then compared with the speed computed by the DUT model"""
        dut = deltaRobot.DeltaMechanics(L=0.524, l=1.244, wb=0.164, up=0.044)

        delta_t = 1.0

        pos_t0 = np.array([0, 0, -1])
        pos_t1 = np.array([0, 0, -1.01])
        delta_d = pos_t1-pos_t0
        tip_speed = delta_d / delta_t

        dut.update_from_new_tip_pos(pos_t0)
        self.assertEqual(dut.valid, True)
        theta_t0 = dut.motor_angles()

        dut.update_from_new_tip_pos(pos_t1)
        self.assertEqual(dut.valid, True)
        theta_t1 = dut.motor_angles()

        delta_theta = theta_t1 - theta_t0
        angular_speed = delta_theta / delta_t

        transposed_tip_speed = np.array([tip_speed]).T

        dut.update_motor_speed_from_tip_speed(transposed_tip_speed)

        self.assertAlmostEqual(dut.theta_prime[0, 0], angular_speed[0], 3)
        self.assertAlmostEqual(dut.theta_prime[1, 0], angular_speed[1], 3)
        self.assertAlmostEqual(dut.theta_prime[2, 0], angular_speed[2], 3)


if __name__ == '__main__':
    unittest.main()
