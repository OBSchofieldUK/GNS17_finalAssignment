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

import src.quadrotor as quad
import src.quadlog as quadlog
import src.animation as ani

from PlotData import plotGraph

drPlot = plotGraph(i=0,XYlim=2)

uavInitPos = np.array([ [1.0,1.2,0.0],\
                        [-1.1,-1.5,0.0],\
                        [1.2,-0.8,0.0],\
                        [-0.6, 1.4,0.0]\
                        ])

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
        droneSet.append(tmp)

    return droneSet

def main():
    agents = initDrones(uavInitPos, 4)
    drPlot.plotDrone(agents)
    pl.pause(0)

if __name__ == '__main__':
    main()
