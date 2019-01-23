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
uavColours = ['r','g','b','k','y','c','m','w']

tf = 120
dt = 5e-2
time = np.linspace(0, tf, tf/dt)
it = 0
frames = 10

# uavInitPos = np.array([ [1.0,1.2,0.0],\
#                         [-1.1,-1.5,0.0],\
#                         [1.2,-0.8,0.0],\
#                         [-0.6, 1.4,0.0]\
#                         ])

# uavInitPos = np.array([ [-0.1,0,0.0],\
#                         [1.3,0.2,0.0],\
#                         [1.0,1,0.0],\
#                         [0.0, 1.0,0.0]\
#                         ])

# uavInitPos = np.array([ [-0.2,0.1,-0.0],\
#                         [1.2,0.2,-0.0],\
#                         [1.2,1.0,-0.0],\
#                         [0.0, 1.0,-0.0]\
#                         ])

uavInitPos = np.array([ [-0.2,0.1,-0.0],\
                        [1.2,0.2,-0.0],\
                        [1.2,0.8,-0.0],\
                        [-0.2, 1.3,-0.0],\
                        [0.5, 1.5,-0.0],\
                        ])

side = 1.75
sideDiag = sqrt(side**2+side**2)

Bsquare = np.array([    [1,0,0,-1,0], \
                        [-1,1,0,0,-1], \
                        [0,-1,1,0,0], \
                        [0,0,-1,1,1] \
                        ])
BHouse = np.array([    [1,0,0,-1,0,0,0], \
                        [-1,1,0,0,-1,0,0], \
                        [0,-1,1,0,0,0,1], \
                        [0,0,-1,1,1,1,0], \
                        [0,0,0,0,0,-1,-1]\
                        ])

Dsquare = np.array([side, side, side, side, sideDiag])
Dhouse = np.array([side, side, side, side, sideDiag,side,side])

Btriang = np.array([[1, 0, -1],[-1, 1, 0],[0, -1, 1]])
dtriang = np.array([side, side, side])

def initDrones(dronePositions,quantity=4):
    # Quadrotor
    m = 0.5 # Kg
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
    droneLogSet=[]

    for i in range(quantity):
        tmp = quad.quadrotor(i, m, l, J, CDl, CDr, kt, km, kw, \
        att_0, pqr_0, uavInitPos[i], v_ned_0, w_0)
        # initYaw = random.randint(0,7)*45
        uavLog = []
        droneSet.append(tmp)
        droneLogSet.append(uavLog)
    return droneSet, droneLogSet

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
    agents, agentLog = initDrones(uavInitPos, 5)
    # FC = formCtl(Bsquare, Dsquare, agents, 0.15, 2)
    FC = formCtl(BHouse, Dhouse, agents, 0.08,2)

    pl.close("all")
    pl.ion()
    fig = pl.figure(0)

    axis3d = fig.add_subplot(111, projection='3d')
    pl.figure(0)

    for t in time:

        FC.run()
        i = 0
        for uav in agents:
            uav.step(dt)
            uavLog = agentLog[i]
            uavLog.append(uav.xyz)
            i += 1

        if it%frames==0 and FC.atAltitude:
            plotDrones(axis3d, agents, t)
            pass
        it+=1

        if checkCrash(agents):
            break
    # plotDrones(axis3d, agents, t)

    # Error Data from the Flight Controller
    data = FC.errLog
    datLen = len(data)
    datWid = len(data[0])
    data = np.concatenate(data)
    data = np.reshape(data, (datLen,datWid))

    matLogs = []
    for uavLogs in agentLog:
        tmp = matReconfig(uavLogs)
        matLogs.append(tmp)

    # Plot errors
    pl.figure(1)
    pl.title('Error Per Step')
    for i in range(datWid):
        pl.plot(data[:, i], color=uavColours[i], label='E'+str(i+1))

    pl.xlabel('Iteration')
    pl.ylabel('offset (M)')
    pl.legend()
    pl.savefig('img/2_House_errorPerIter',format="svg")

    # Plot Movement
    pl.figure(2)
    pl.title("2D Position [m]")
    j = 0
    for uav in matLogs:
        pl.scatter(uav[0,0], uav[0,1], color=uavColours[j], marker='+',s=50, label='q'+str(j+1))
        pl.plot(uav[:,0], uav[:,1], color=uavColours[j])
        j +=1

    for j in range(FC.numEdges):
        edgeTail = 0
        edgeHead = 0
        for i in range(FC.numAgents):
            action = FC.CoIncMatx[i,j]
            if action == -1:
                edgeTail = i
            elif action == 1:
                edgeHead = i
        if edgeHead != edgeTail:
            x1 = FC.dronePositions[edgeHead,0]
            x2 = FC.dronePositions[edgeTail,0]
            y1 = FC.dronePositions[edgeHead,1]
            y2 = FC.dronePositions[edgeTail,1]
            # print(x1, x2)
            pl.plot([x1,x2],[y1,y2],'r--',lw=2)

    pl.axis('equal')
    pl.xlabel("East")
    pl.ylabel("South")
    pl.legend()
    pl.grid()
    pl.savefig('img/2_House_FormationPositions',format="svg")
    pl.pause(0)
if __name__ == '__main__':
    main()
