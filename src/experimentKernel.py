#!/usr/bin/python3

import time
from servo import servo
from cnc.deltaRobot import *


s0 = servo.ServoKernel(0)
s1 = servo.ServoKernel(1)
s2 = servo.ServoKernel(2)

w = np.arange(0, 2.0 * math.pi, 2.0 * math.pi / 300)
radius = np.linspace(0.001, 0.02, 20)

meca = DeltaMechanics(L=0.02, l=0.04, wb=0.02, up=0.00922)

zero_offset = -0.16

startPos=90 * math.pi / 180

s0.move_to_angle(-startPos + zero_offset)
s1.move_to_angle(-startPos + zero_offset)
s2.move_to_angle(-startPos + zero_offset)
time.sleep(1)

pos = [ 0, 0, 0]
try:
    while True:
        for r in radius:
            for i in range(0, len(w)):
                pos[0] = r * np.sin(w[i])
                pos[1] = r * np.cos(w[i])
                pos[2] = -0.04
                meca.update_from_new_tip_pos(pos)
                #print(meca.theta)
                theta = meca.motor_angles()
                s0.move_to_angle(-theta[0]+zero_offset)
                s1.move_to_angle(-theta[1]+zero_offset)
                s2.move_to_angle(-theta[2]+zero_offset)
                #time.sleep(0.01)
except KeyboardInterrupt:
    s0.close()
    s1.close()
    s2.close()
    print("Graceful stop after keyboard interrupt")
