#!/usr/bin/python3

import numpy as np
import time
import math
from servo import servo
from  deltaRobot import *


s0 = servo.ServoPython(0)
s1 = servo.ServoPython(1)
s2 = servo.ServoPython(2)

w = np.arange(0, 2.0 * math.pi, 2.0 * math.pi / 200)
radius = np.linspace(0.001,0.015,20) 

meca = DeltaMechanics( L = 0.02, l=0.04, wb=0.02, up=0.00922)

zero_offset = 1.4

startPos=15.0 * math.pi / 180;

s0.move_to_angle(-startPos + zero_offset)
s1.move_to_angle(-startPos + zero_offset)
s2.move_to_angle(-startPos + zero_offset)
time.sleep(1)

pos = [ 0, 0, 0]
while True:
    for r in radius:
        for i in range(0, len(w)):
            pos[0] = r * np.sin(w[i])
            pos[1] = r * np.cos(w[i])
            pos[2] = -0.04
            meca.update_from_new_tip_pos(pos)
            s0.move_to_angle(-meca.theta[0]+zero_offset)
            s1.move_to_angle(-meca.theta[1]+zero_offset)
            s2.move_to_angle(-meca.theta[2]+zero_offset)
            time.sleep(0.001)
