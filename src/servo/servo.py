#!/usr/bin/python3
from .PWM import PWM
import time
import os.path


class Servo:
    init = False
    channel = 0

    def __init__(self, channel):
        self.channel = channel
        if not self.init:
            self.init_controller()

    def init_controller(self):
        raise NotImplementedError

    def move_to_angle(self, radian):
        raise NotImplementedError


class ServoPython(Servo):
    pwm = PWM(0x40)
    init = False
    PULSE_MIN = 150
    PULSE_MAX = 500

    def init_controller(self):
        self.pwm.set_pwm_freq(60)
        self.init = True

    def move_to_angle(self, radian):
        pulse = 130.57*radian + 150.0
        if pulse < self.PULSE_MIN:
            pulse = self.PULSE_MIN
        elif pulse > self.PULSE_MAX:
            pulse = self.PULSE_MAX
        self.pwm.set_pwm(self.channel, 0, int(pulse))


class ServoKernel(Servo):
    period_ns = 20000000.0
    servoClassPath = '/sys/class/pwm/pwmchip0'
    model_slope = 531034.0
    model_intercept = 1550000.0 
    PULSE_MIN = 600000.0
    PULSE_MAX = 2500000.0
    duty_cycle_path = ''

    def init_controller(self):
        export_path = "{:s}/export".format(self.servoClassPath)
        pwm_path = "{:s}/pwm{:d}".format(self.servoClassPath, self.channel)
        if not os.path.exists(pwm_path):
            try:
                with open(export_path, 'w') as fh:
                    fh.writelines("{:d}".format(self.channel))
            except OSError as err:
                print("Error in writing to " + export_path)
                print(self.channel)
                print(err)
                exit(0)

        period_path = "{:s}/period".format(pwm_path)
        self.duty_cycle_path = "{:s}/duty_cycle".format(pwm_path)
        with open(period_path, 'w') as fh:
            fh.writelines("{:d}".format(int(self.period_ns)))

    def move_to_angle(self, radian):
        duty_cycle = self.model_slope*radian + self.model_intercept
        if duty_cycle < self.PULSE_MIN:
            duty_cycle = self.PULSE_MIN
        elif duty_cycle > self.PULSE_MAX:
            duty_cycle = self.PULSE_MAX
        with open(self.duty_cycle_path, 'w') as fh:
            #print(duty_cycle)
            fh.writelines("{:d}".format(int(duty_cycle)))


if __name__ == '__main__':
    s0 = ServoKernel(0)
    s1 = ServoKernel(2)
    wt = 0.20
    while True:
        s0.move_to_angle(0)
        time.sleep(wt)
        s0.move_to_angle(2)
        time.sleep(wt)
