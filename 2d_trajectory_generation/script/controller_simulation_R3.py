# -*- coding: utf-8 -*-
"""
Created on Mon Sep 21 10:11:44 2020

@author: ind
"""

import numpy as np
import matplotlib.pyplot as plt
from numpy import sin, cos, arctan2, arctan, tan


t0 = 0; tn = 1.0; interval = 51
t = np.linspace(t0, tn, interval)
delt = (tn-t0)/(interval-1)
delt_bar = delt/2.0
xr = 1+t+t**2+t**3; yr = 1+t+t**2+t**3
thr = arctan2(1+2*t+3*t**2, 1+2*t+3*t**2)
x1r = xr
x3r = yr
x2r = (1+2*t+3*t**2)/(1+2*t+3*t**2)

x1, x2, x3 = [], [], []
u1, u21, u22 = [], [], []
x, y, th = [], [], []
lambda1 = 0.8; lambda2 = 0.8; lambda3 = 0.5
for i in range(len(t)): 
    if i == 0:
        
        u1.append(0); u21.append(0); u22.append(0)
        x.append(0); y.append(0); th.append(np.pi)
        x1.append(x[-1]); x2.append(tan(th[-1])); x3.append(y[-1])
      
    else: 

        #x1r.append(xr[i]); x3r.append(yr[i]); x2r.append(tan(thr[i]))
        u1.append(1/delt * (x1r[i] - x1[-1]))
        A_delt = np.array([[1, 0], [delt*u1[-1], 1]])
        B_delt = np.array([[delt_bar, delt_bar], [3.0 * delt_bar**2 * u1[-1] / 2, delt_bar**2 * u1[-1] / 2]])
        Xi_f = np.array([x2r[i], x3r[i]]); Xi_0 = np.array([x2[-1], x3[-1]])
        #print '**',Xi_f, x2r[i]
        u2 = np.dot(np.linalg.inv(B_delt), (Xi_f - np.dot(A_delt,Xi_0)))
        u21.append(u2[0]); u22.append(u2[1])
        x1.append(x1[-1] + u1[-1] * delt)
        Xi = np.dot(A_delt, Xi_0) + np.dot(B_delt, u2)
        x2.append(Xi[0]); x3.append(Xi[1])
        x.append(x1[-1]); y.append(x3[-1]); th.append(arctan(x2[-1]))
        #print x1[-1], x3[-1], x2[-1]
    
plt.figure()
plt.plot(t, yr, 'b', linewidth = 2, label='Ref')    
plt.plot(t, y, 'r', linewidth = 2, label='Actual')  
    
    