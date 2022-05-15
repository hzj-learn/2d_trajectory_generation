# -*- coding: utf-8 -*-
"""
Created on Sat Sep 12 11:28:49 2020

@author: ind
"""

import numpy as np
import matplotlib.pyplot as plt
from numpy import sin, cos, arctan

t0 = 0; tn = 4.0; interval = 4001
tt = np.linspace(t0, tn, interval)
delt = (tn-t0)/(interval-1)
xr = 50*sin(2*tt); yr = tt
k1 = 14
k2 = 100
e = 0.3
x, y, theta = [], [], []
x_dot, y_dot, vel = [], [], []

for t in tt: 
    if t == 0: 
        x.append(10); y.append(2); theta.append(np.pi/4)
        x_dot.append(100); y_dot.append(1)
        vel.append(np.sqrt(100**2 + 1)) 
    else: 
        
        xd_dot = 100 * cos(2*t)
        yd_dot = 1.0
        xd_ddot = - 200* sin(2*t)
        yd_ddot = 0.0
        
        
        v1 = xd_ddot - k2 * (x[-1] - 50*sin(2*t)) - k1 * (x_dot[-1] - xd_dot)
        v2 = yd_ddot - k2 * (y[-1] - t) - k1 * (y_dot[-1] - yd_dot)
        
        u1 = v1 * cos(theta[-1]) + v2 * sin(theta[-1])
        u2 = (v2 * cos(theta[-1]) - v1 * sin(theta[-1]))/vel[-1]
        
        vel.append(vel[-1] + u1 * delt)
        theta.append(theta[-1] + u2 * delt)
        x_dot.append(vel[-1] * cos(theta[-1]))
        y_dot.append(vel[-1] * sin(theta[-1]))
        xk = x[-1] + vel[-1] * cos(theta[-1]) * delt
        yk = y[-1] + vel[-1] * sin(theta[-1]) * delt       
        x.append(xk); y.append(yk)    
 

#print x
plt.figure()
plt.plot(tt, xr, 'b', linewidth = 2, label='Ref')
plt.plot(tt, x, 'r', linewidth = 2, label='Actual')
plt.legend(loc = 1, fontsize = 16)
plt.xlabel('Time(s)', fontsize = 16)
plt.ylabel('Trajectory', fontsize = 16)
plt.xticks(size = 16);plt.yticks(size = 16)
plt.show()