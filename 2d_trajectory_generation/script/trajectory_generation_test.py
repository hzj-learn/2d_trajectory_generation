#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
generate polynonial trajectory 
@author: indrajeet
"""

import numpy as np 
from matplotlib import pyplot as plt 
import time 


class polynomial_trajetory(object):
    def __init__(self): 
        """initializes variables"""
        self.start = np.array([0.0,0.0])
        #self.end = np.array([8,8])
        #self.xic = np.array([self.start[0], 0, 0])
        #self.yic = np.array([self.start[1], 0, 0])
        #self.xfc = np.array([self.end[0], 0, 0])
        #self.yfc = np.array([self.end[1], 0, 0])
        #self.T = 3.0
 

       
    def coefficients(self, start_conditions , end_conditions, T): 
        x0 = start_conditions[0]
        v0 = start_conditions[1]
        a0 = start_conditions[2]
        xT = end_conditions[0]
        vT = end_conditions[1]
        aT = end_conditions[2]
        delx = xT - (x0 + v0 * T + 0.5 * a0 * T**2)
        delv  = vT - (v0 + a0 * T)
        dela = aT - a0 
        dels = np.array([delx, delv, dela])
        
        A = (1/T**5) * np.array([[720.0, -360*T, 60*T**2], [360*T, -168*T**2, 24*T**3], [-60*T**2, 24*T**3, -3*T**4]])
        
        first_three = np.dot(A, dels)
        c1 = first_three[0]; c2 = first_three[1]; c3 = first_three[2]
        c4 = a0; c5 = v0; c6 = x0
        
        return [c1, c2, c3, c4, c5, c6]
       
    def plot(self, c, d, T): 
        t = np.linspace(0, T, 101)
        x = c[0]*t**5/120 - c[1]*t**4/24 - c[2]*t**3/6 + c[3]*t**2/2 + c[4]*t + c[5]
        y = d[0]*t**5/120 - d[1]*t**4/24 - d[2]*t**3/6 + d[3]*t**2/2 + d[4]*t + c[5]
        vx = c[0]*t**4/24 - c[1]*t**3/6 - c[2]*t**2/2 + c[3]*t + c[4]
        vy = d[0]*t**4/24 - d[1]*t**3/6 - d[2]*t**2/2 + d[3]*t + d[4]
        ax = c[0]*t**3/6 - c[1]*t**2/2 - c[2]*t + c[3]
        ay = d[0]*t**3/6 - d[1]*t**2/2 - d[2]*t + d[3]
        
       
        plt.plot(x, y, 'r', linewidth = 2, label='trajectory')

        plt.xlabel('x(m)', fontsize = 16)
        plt.ylabel('y(m)', fontsize = 16)
        plt.xticks(size = 16);plt.yticks(size = 16)
        

        #plt.plot(t, vx, 'g', linewidth = 2, label='vx')
        #plt.plot(t, vy, 'b', linewidth = 2, label='vy')

        #plt.xlabel('t(sec)', fontsize = 16)
        #plt.ylabel('velocity(m/s)', fontsize = 16)
        #plt.xticks(size = 16);plt.yticks(size = 16)
        

        #plt.plot(t, ax, 'g', linewidth = 2, label='ax')
        #plt.plot(t, ay, 'b', linewidth = 2, label='ay')

        #plt.xlabel('t(sec)', fontsize = 16)
        #plt.ylabel('acceleration(m/s^2)', fontsize = 16)
        #plt.xticks(size = 16);plt.yticks(size = 16)
        #plt.show()
        
    def generate_trajectory(self): 
        t = time.time()
        av_speed = 1.0
        nplanes = 5
        range_min = 1.0
        range_max = 5.0
        half_angle = 60
        d = (range_max - range_min)/(nplanes-1)
        #xx = range_max * tan(45*np.pi/180)
        #yy = range_max * tan(45*np.pi/180)
        #resolution = 0.25;
        plt.figure()
        for i in range(nplanes): 
            r  = range_min + d * 4
            #xmax  = range_min + d * i
            #ymax = xmax * np.tan(45*PI/180)
            #Npointsy = int(2 * ymax / resolution)
            for j in range(-half_angle, half_angle+1, 5):
                end = np.array([self.start[0] + r*np.cos(40*np.pi/180), self.start[1] + r*np.sin(40*np.pi/180)])
                xic = np.array([self.start[0], 0, 0])
                yic = np.array([self.start[1], 0, 0])
                xfc = np.array([end[0], 0, 0])
                yfc = np.array([end[1], 0, 0]) 
                #T = np.linalg.norm(end-self.start)/av_speed
                T = np.sqrt((end[0]-self.start[0])**2 + (end[1]-self.start[1])**2)/av_speed
                x_coefficients = self.coefficients(xic, xfc, T)
                y_coefficients =  self.coefficients(yic, yfc, T)
                self.plot(x_coefficients, y_coefficients, T)
        plt.show()
        print 'total time:', time.time()-t
    


if __name__ == '__main__':

    f = polynomial_trajetory()
    f.generate_trajectory()
