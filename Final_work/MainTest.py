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

uav1 = []
uav2 = []
uav3 = []
uav4 = []

tf = 60
dt = 5e-2
time = np.linspace(0, tf, tf/dt)
it = 0
frames = 10

# uavInitPos = np.array([ [1.0,1.2,0.0],\
#                         [-1.1,-1.5,0.0],\
#                         [1.2,-0.8,0.0],\
#                         [-0.6, 1.4,0.0]\
#                         ])

uavInitPos = np.array([ [-0.1,0,0.0],\
                        [1.3,0.2,0.0],\
                        [1.0,1,0.0],\
                        [0.0, 1.0,0.0]\
                        ])

side = 1
sideDiag = sqrt(side**2+side**2)
#
# Bsquare = np.array([    [1,0,0,-1,0,1], \
#                         [-1,1,0,0,-1,0], \
#                         [0,-1,1,0,0,-1], \
#                         [0,0,-1,1,1,0] \
#                         ])

Bsquare = np.array([    [1,0,0,-1,0], \
                        [-1,1,0,0,-1], \
                        [0,-1,1,0,0], \
                        [0,0,-1,1,1] \
                        ])

Dsquare = np.array([side, side, side, side, sideDiag])

Btriang = np.array([[1, 0, -1],[-1, 1, 0],[0, -1, 1]])
dtriang = np.array([side, side, side])




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

def plotDrones(graph, droneSet, t):
    uavColours = ['r','g','b','k','y','c','m','w']
    count = 0
    graph.cla()
    for uav in droneSet:
        ani.draw3d(graph, uav.xyz, uav.Rot_bn(), uavColours[count])
        if count <= len(uavColours):
            count +=1
        else:
            count = 0

    graph.set_xlim(-5, 5)
    graph.set_ylim(-5, 5)
    graph.set_zlim(0, 10)
    graph.set_xlabel('South [m]')
    graph.set_ylabel('East [m]')
    graph.set_zlabel('Up [m]')
    graph.set_title("Time %.3f s" %t)
    pl.pause(0.001)
    pl.draw()

def matReconfig(lista):
    lislen = len(lista)
    lista = np.concatenate(lista)
    out = np.resize(lista,(lislen,3))
    return out

def checkCrash(drones):
    crash = False
    for uav in drones:
        if uav.crashed:
            crash = True
    return crash

def main():
    it = 0
    agents = initDrones(uavInitPos, 4)
    # FC = formCtl(Btriang ,dtriang, agents, -5)
    FC = formCtl(Bsquare, Dsquare, agents, -5)

    pl.close("all")
    pl.ion()
    fig = pl.figure(0)

    axis3d = fig.add_subplot(111, projection='3d')
    pl.figure(0)

    xyz_D = np.array([5, 3, -10])

    for t in time:
        XYDist, droneVel = FC.run()
        print(XYDist, end='\n\n')
        # print()
        for i in range(len(agents)):
            xyz_D = np.array([XYDist[i,0], XYDist[i,1], -5])
            # agents[i].set_xyz_ned_lya(xyz_D)
            agents[i].set_xyz_ned_lya(xyz_D)
            # agents[i].set_v_2D_alt_lya([droneVel[i,0], droneVel[i,1]], alt_d=-5)
            # agents[i].set_xyz_ned_lya([XYDist[i,0], XYDist[i,1]], -5)

        for uav in agents:
            uav.step(dt)

        uav1.append(agents[0].xyz)
        uav2.append(agents[1].xyz)
        uav3.append(agents[2].xyz)
        uav4.append(agents[3].xyz)

        if it%frames==0:
            plotDrones(axis3d, agents, t)
            pass
        it+=1

        if checkCrash(agents):
            break

    data = FC.errLog
    datLen = len(data)
    datWid = len(data[0])
    data = np.concatenate(data)
    data = np.reshape(data, (datLen,datWid))

    uavA = matReconfig(uav1)
    uavB = matReconfig(uav2)
    uavC = matReconfig(uav3)
    uavD = matReconfig(uav4)

    pl.figure(1)
    pl.title('error per iteration')
    pl.plot(data[:, 0], color='r', label='q1')
    pl.plot(data[:, 1], color='g', label='q2')
    pl.plot(data[:, 2], color='b', label='q3')
    pl.plot(data[:, 3], color='y', label='q4')
    pl.xlabel('iterations')
    pl.ylabel('offset (M)')
    pl.legend()


    pl.figure(2)
    pl.title("2D Position [m]")
    pl.scatter(uavA[0,0], uavA[0,1], color='r', marker='+',s=40, label='q1')
    pl.scatter(uavB[0,0], uavB[0,1], color='g', marker='+',s=40, label='q2')
    pl.scatter(uavC[0,0], uavC[0,1], color='b', marker='+',s=40, label='q3')
    pl.scatter(uavD[0,0], uavD[0,1], color='y', marker='+',s=40, label='q4')
    pl.plot(uavA[:,0], uavA[:,1], color='r')
    pl.plot(uavB[:,0], uavB[:,1], color='g')
    pl.plot(uavC[:,0], uavC[:,1], color='b')
    pl.plot(uavD[:,0], uavD[:,1], color='y')
    pl.axis('equal')

    pl.xlabel("East")
    pl.ylabel("South")
    pl.legend()
    pl.pause(0)
if __name__ == '__main__':
    main()
