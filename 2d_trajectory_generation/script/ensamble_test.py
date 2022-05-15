# -*- coding: utf-8 -*-
"""
Created on Tue Sep 22 17:17:31 2020

@author: ind
"""

import numpy as np
import matplotlib.pyplot as plt
from numpy import sin, cos, arctan2, arctan, tan

fov = 90
rmax = 5.0
rmin = 2.0
nplanes = 5
xf = [18.0, 18.0]
d = (rmax-rmin)/(nplanes-1)
angle = 0.5 * fov * np.pi/180;
#x0 = 0.417551; y0 = 0.407273; yaw0 = 0.773036
x0 = 0; y0 = 0; yaw0 = 0
resol = 0.25
pp1, pp2, dist = [], [], []
#for i in range(nplanes):
#    xmax = rmin + d*i
#    ymax = xmax * np.tan(angle)
#    npoints = int(2 * ymax / resol)
#    for j in range(npoints): 
#        x = xmax
#        y = - ymax + j * resol
#        p1 = x0 + x * cos(yaw0) - y * sin(yaw0);
#        p2 = y0 + x * sin(yaw0) + y * cos(yaw0);
#        distance = np.sqrt((xf[0] - p1)**2 + (xf[1] - p2)**2)
#        dist.append(distance)
#        pp1.append(p1); pp2.append(p2)

#k = dist.index(min(dist))
#plt.figure()
#plt.scatter(pp1, pp2, c= 'r')
#plt.scatter(pp1[k], pp2[k])
npoints_in_arc = 10
angles = np.linspace(-angle, angle, npoints_in_arc)
print angles

for i in range(nplanes): 
    r = rmin + d*i
    for j in angles: 
        x = r*cos(j)
        y = r*sin(j)
        p1 = x0 + x * cos(yaw0) - y * sin(yaw0);
        p2 = y0 + x * sin(yaw0) + y * cos(yaw0);
        distance = np.sqrt((xf[0] - p1)**2 + (xf[1] - p2)**2)
        dist.append(distance)
        pp1.append(p1); pp2.append(p2)

k = dist.index(min(dist))
plt.figure()
plt.scatter(pp1, pp2, c= 'r')
plt.scatter(pp1[k], pp2[k])        

