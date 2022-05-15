# -*- coding: utf-8 -*-
"""
Created on Sat Sep 12 11:28:49 2020

@author: ind
"""

import numpy as np
import matplotlib.pyplot as plt
from numpy import sin, cos, arctan

t0 = 0; tn = 10.0; interval = 101
tt = np.linspace(t0, tn, interval)
delt = (tn-t0)/(interval-1)
xr = tt; yr = tt
k = 0.75
e = 0.6
x, y, theta = [], [], []
x.append(0); y.append(-0.0); theta.append(0)

for t in tt: 
    zr1_dot = 1 #+ e*sin(t)/(1+(cos(t))**2) * sin(arctan(cos(t)))
    zr2_dot = 1 #- e*sin(t)/(1+(cos(t))**2) * cos(arctan(cos(t)))

    u1 = zr1_dot - k * (x[-1] + e * cos(theta[-1]) - (t + e * cos(arctan(1))))
    u2 = zr2_dot - k * (y[-1] + e * sin(theta[-1]) - (t + e * sin(arctan(1))))
    xk = x[-1] + (u1*cos(theta[-1]) + u2*sin(theta[-1]))* cos(theta[-1]) * delt
    yk = y[-1] + (u1*cos(theta[-1]) + u2*sin(theta[-1]))* sin(theta[-1]) * delt
    x.append(xk); y.append(yk)
    w = theta[-1] + ((-u1 * sin(theta[-1]) + u2 * cos(theta[-1]))*delt/e)
    theta.append(w)

#print x
plt.figure()
#plt.plot(xr, yr, 'b', linewidth = 2, label='Ref')
#plt.plot(x, y, 'r', linewidth = 2, label='Actual')
plt.plot(tt, yr[:101], 'b', linewidth = 2, label='Ref')
plt.plot(tt, y[:101], 'r', linewidth = 2, label='Actual')
plt.plot(tt,theta[:101], 'k', linewidth = 2, label='theta')
plt.legend(loc = 2, fontsize = 16)
plt.xlabel('Time(s)', fontsize = 16)
plt.ylabel('Trajectory', fontsize = 16)
plt.xticks(size = 16);plt.yticks(size = 16)
plt.show()