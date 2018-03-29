#!/usr/bin/python3

import numpy as np
import time
import math
from servo import servo

s0 = servo.ServoKernel(0)
s1 = servo.ServoKernel(1)
s2 = servo.ServoKernel(2)

w = np.arange(0, 2.0 * math.pi, 2.0 * math.pi / 200)

amplitude = math.radians(30.0)

theta0 = amplitude * np.sin(w) + 1
theta1 = amplitude * np.sin(w + 2.0 * math.pi / 3.0) + 1
theta2 = amplitude * np.sin(w + 4.0 * math.pi / 3.0) + 1

s0.move_to_angle(0)
s1.move_to_angle(0)
s2.move_to_angle(0)
time.sleep(1)

while True:
    for i in range(0, len(w)):
        s0.move_to_angle(theta0[i])
        s1.move_to_angle(theta1[i])
        s2.move_to_angle(theta2[i])
        # time.sleep(0.0001)
