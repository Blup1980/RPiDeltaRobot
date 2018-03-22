#!/usr/bin/python3
from .PWM import PWM
import time

PULSE_MIN = 150  # Min pulse length out of 4096
PULSE_MAX = 500  # Max pulse length out of 4096


class Servo:
    pwm = PWM(0x40)
    init = False

    def __init__(self, channel):
        if not self.init:
            self.init_controller()
        self.channel = channel

    def init_controller(self):
        self.pwm.set_pwm_freq(60)
        self.init = True

    def move_to_angle(self, radian):
        pulse = 130.57*radian + 150.0
        if pulse < PULSE_MIN:
            pulse = PULSE_MIN
        elif pulse > PULSE_MAX:
            pulse = PULSE_MAX
        self.pwm.set_pwm(self.channel, 0, int(pulse))


if __name__ == '__main__':
    s0 = Servo(0)
    s1 = Servo(2)
    wt = 0.20
    while True:
        s0.move_to_angle(0)
        time.sleep(wt)
        s0.move_to_angle(2)
        time.sleep(wt)
