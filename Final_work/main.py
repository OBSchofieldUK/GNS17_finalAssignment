#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Guidance and Navigation Systems - Final Assignment
Based on the work by Noether PyCopter:

Assignment 3 - Relative Distance-Based Formation Control.

"""
from scipy import linalg as la
import matplotlib.pyplot as pl
import numpy as np
import random
from math import (radians, degrees, sqrt)

import src.quadrotor as quad
import src.quadlog as quadlog
import src.animation as ani

from PlotData import plotGraph
from obsFormation import formCtl

drPlot = plotGraph(i=0,XYlim=2)


tf = 60
dt = 5e-2
time = np.linspace(0, tf, tf/dt)
it = 0
frames = 100

uavInitPos = np.array([ [1.0,1.2,0.0],\
                        [-1.1,-1.5,0.0],\
                        [1.2,-0.8,0.0],\
                        [-0.6, 1.4,0.0]\
                        ])

side = 1
sideDiag = sqrt(side**2+side**2)

Bsquare = np.array([    [-1,0,0,1,0], \
                        [1,-1,0,0,1], \
                        [0,1,-1,0,0], \
                        [0,0,1,-1,-1] \
                        ])

Dsquare = np.array([side, side, side, side, sideDiag])

def initDrones(dronePositions,quantity=4):
    # Quadrotor
    m = 0.65 # Kg
    l = 0.23 # m
    Jxx = 7.5e-3 # Kg/m^2
    Jyy = Jxx
    Jzz = 1.3e-2
    Jxy = 0
    Jxz = 0
    Jyz = 0
    J = np.array([[Jxx, Jxy, Jxz], \
                  [Jxy, Jyy, Jyz], \
                  [Jxz, Jyz, Jzz]])
    CDl = 9e-3
    CDr = 9e-4
    kt = 3.13e-5  # Ns^2
    km = 7.5e-7   # Ns^2
    kw = 1/0.18   # rad/s

    att_0 = np.array([0.0, 0.0, 0.0])
    pqr_0 = np.array([0.0, 0.0, 0.0])
    v_ned_0 = np.array([0.0, 0.0, 0.0])
    w_0 = np.array([0.0, 0.0, 0.0, 0.0])

    droneSet=[]

    for i in range(quantity):
        tmp = quad.quadrotor(i, m, l, J, CDl, CDr, kt, km, kw, \
        att_0, pqr_0, uavInitPos[i], v_ned_0, w_0)
        # initYaw = random.randint(0,7)*45
        droneSet.append(tmp)
    return droneSet

def main():
    agents = initDrones(uavInitPos, 4)
    FC = formCtl(Bsquare ,Dsquare, agents, -5)
    for t in time:
        XYDist = FC.run()
        # print(XYDist)
        for i in range(len(agents)):
            xyz_D = np.array([XYDist[i,0], XYDist[i,1], -5])
            agents[i].set_xyz_ned_lya(xyz_D)
        for uav in agents:
            uav.step(dt)
        if it%frames == 0:
            drPlot.plotDrone(agents)

if __name__ == '__main__':
    main()
