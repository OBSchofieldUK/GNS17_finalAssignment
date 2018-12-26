#!/usr/bin/env python3
import matplotlib.pyplot as pl
import numpy as np
import math
import src.animation as ani
uavColours = ['r','g','b','k','y','c','m','w']

class plotGraph(object):
    def __init__(self, i=0,XYlim=5):
        self.figNum=i
        self.fig = pl.figure(self.figNum)
        self.axis3d = self.fig.add_subplot(111,projection='3d')
        self.axis3d.set_xlim(-XYlim, XYlim)
        self.axis3d.set_ylim(-XYlim, XYlim)
        self.axis3d.set_zlim(0, 15)
        self.axis3d.set_xlabel('South [m]')
        self.axis3d.set_ylabel('East [m]')
        self.axis3d.set_zlabel('Up [m]')

    def plotDrone(self, drones):
        # self.axis3d.cla()
        i = 0
        for uav in drones:
            ani.draw3d(self.axis3d, uav.xyz, uav.Rot_bn(),uavColours[i])
            if i != len(uavColours):
                i=i+1
            else:
                i = 0
        pl.pause(0.001)
        pl.draw()

    # def drawGraph(self)
