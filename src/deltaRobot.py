#!/usr/bin/python3

import numpy as np
import math


class DeltaMechanics:
    def __init__(self, L, l, wb, up):
        self.L = L
        self.l = l
        self.wb = wb
        self.up = up

        self.sp = 3.0*self.up / np.sqrt(3.0)
        self.a = self.wb - self.up
        self.b = self.sp/2.0 - np.sqrt(3.0)*self.wb/2.0
        self.c = self.up/2.0 - self.wb/2.0

        self.theta = np.array([0.0, 0.0, 0.0])
        self.valid = True

    def update_from_new_tip_pos(self, tip_pos):
        x = tip_pos[0]
        y = tip_pos[1]
        z = tip_pos[2]

        alpha = x**2 + y**2 + z**2 + self.L**2 - self.l**2

        e1 = 2.0*self.L*(y + self.a)
        f1 = 2.0*z*self.L
        g1 = alpha + self.a**2 + 2.0*y*self.a

        e2 = -self.L * (np.sqrt(3.0)*(x+self.b)+y+self.c)
        f2 = 2.0*z*self.L
        g2 = alpha + self.b**2 + self.c**2 + 2.0*(x*self.b + y*self.c)

        e3 = self.L * (np.sqrt(3.0) * (x-self.b) - y - self.c)
        f3 = 2.0*z*self.L
        g3 = alpha + self.b**2 + self.c**2 + 2.0*(-x*self.b + y*self.c)

        self.theta[0] = self._solve_ipk(e1, f1, g1)
        self.theta[1] = self._solve_ipk(e2, f2, g2)
        self.theta[2] = self._solve_ipk(e3, f3, g3)

        if any(np.isnan(self.theta)):
            self.valid = False
        else:
            self.valid = True

    def _solve_ipk(self, e, f, g):
        delta = e**2 + f**2 - g**2
        if delta >= 0:
            pos_sol_t = (-f + np.sqrt(delta)) / (g-e)
            neg_sol_t = (-f - np.sqrt(delta)) / (g-e)

            pos_sol = 2.0 * math.atan(pos_sol_t)
            neg_sol = 2.0 * math.atan(neg_sol_t)
            return self._best_angle(pos_sol, neg_sol)
        else:
            return np.nan

    def _best_angle(self, angle1, angle2):
        r1 = -self.wb - self.L*math.cos(angle1)
        h1 = -self.L*math.cos(angle1)
        d1 = r1**2 + h1**2

        r2 = -self.wb - self.L*math.cos(angle2)
        h2 = -self.L * math.cos(angle2)
        d2 = r2 ** 2 + h2 ** 2

        if d1 > d2:
            return angle1
        else:
            return angle2


if __name__ == '__main__':
    mech = DeltaMechanics(L=0.524, l=1.244, wb=0.164, up=0.044)
    mech.update_from_new_tip_pos(np.array([0, 0, -0.9]))
    print(mech.theta)
    print(mech.valid)
    mech.update_from_new_tip_pos(np.array([0.3, 0.5, -5.1]))
    print(mech.theta)
    print(mech.valid)
