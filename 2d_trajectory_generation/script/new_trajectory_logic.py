# -*- coding: utf-8 -*-
"""
Created on Wed Sep 16 22:08:56 2020

@author: ind
"""

import numpy as np
import matplotlib.pyplot as plt
from numpy import sin, cos, tan

t0 = 0.0; tf = 1.0; n = 101
t = np.linspace(t0, tf, n)
p = (10/tf**3) * t**3 - (15/tf**4) * t**4 + (6/tf**5) * t**5
plt.figure()
plt.plot(t, p, 'r', linewidth = 2, label = "t-p curve")
plt.legend(loc = 2, fontsize = 16)
plt.xlabel('t(sec)', size = 18)
plt.ylabel('p', size = 18)
plt.xticks(fontsize = 16); plt.yticks(fontsize = 16)

x0 = 0; y0 = 0; th0 = 0.0
xf = 5.0; yf = 1.0; thf = -np.pi/2#np.arctan(yf/xf)

delx = xf - x0; dely = yf - y0
a0 = x0; b0 = y0
a1 = 2 * (dely - delx * tan(thf))/(tan(th0) - tan(thf))
b1 = a1 * tan(th0)
a2 = delx - a1
b2 = 0.5 * a1 * (tan(thf) - tan(th0)) + a2 * tan(thf)

x = a0 + a1 * p**2 + a2 * p**3
y = b0 + b1 * p**2 + b2 * p**3

plt.figure()
plt.plot(t, x, 'g', linewidth = 2, label = "x")
plt.plot(t, y, 'b', linewidth = 2, label = "y")
plt.legend(loc = 2, fontsize = 16)
plt.xlabel('t(sec)', size = 18)
plt.ylabel(' x and y', size = 18)
plt.xticks(fontsize = 16); plt.yticks(fontsize = 16)

plt.figure()
plt.plot(x, y, 'r', linewidth = 2, label = "x-y curve")
plt.legend(loc = 2, fontsize = 16)
plt.xlabel('x', size = 18)
plt.ylabel('y', size = 18)
plt.xticks(fontsize = 16); plt.yticks(fontsize = 16)