# import matplotlib.pyplot as pl
#
# import quadrotor as quad
# import formation_distance as form
# import quadlog
# import animation as ani

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from math import sqrt
from scipy import linalg as la



def calcVelocity():
    pass




# Motion
mu = 0e-2*np.array([1, 1, 1, 1])
tilde_mu = 0e-2*np.array([1, 1, 1, 1])

class formCtl(object):
    """docstring for formCtl."""
    def __init__(self, B, matDists, swarm, inAlt):
        self.desiredAlt = inAlt
        self.CoIncMatx = B
        self.desDistMatx = matDists
        self.numAgents, self.numEdges = self.CoIncMatx.shape
        self.droneSet = swarm
        self.dronePositions = np.zeros((1,self.numAgents*2))
        self.zDetails = []
        self.getPos()

    def calcZ(self, tailPos, headPos, edgeNum):
        Z = tailPos - headPos
        normZ = la.norm(Z)
        error = la.norm(Z) - self.desDistMatx[edgeNum]
        val = (Z/normZ)*error
        self.zDetails.append([Z, error])
        return val

    def getPos(self):
        tmpPos = []
        for drone in self.droneSet:
            tmpPos.append(drone.xyz[0:2])
        self.dronePositions = np.concatenate(tmpPos)
        self.dronePositions = np.reshape(self.dronePositions, (self.numAgents,2))
        # print(self.dronePositions)


    def calcZmatX(self):
        vTemp = []# np.array((self.numEdges,2))
        pDot = np.array((1,self.numAgents))
        self.zDetails = []
        for j in range(self.numEdges):
            edgeTail = 0
            edgeHead = 0
            for i in range(self.numAgents):
                action = self.CoIncMatx[i,j]
                if action ==-1:
                    edgeTail = i
                elif action == 1:
                    edgeHead = i
            vTemp.append(self.calcZ(self.dronePositions[edgeTail], self.dronePositions[edgeHead],j))
        self.droneZ = np.concatenate(vTemp)
        self.droneZ = np.reshape(self.droneZ, (self.numEdges,2))


    def calcForm(self):
        outputDist = np.zeros((self.numAgents,2))
        for i in range(self.numAgents):
            # print(i+1, end='\t')
            for j in range(self.numEdges):
                action = self.CoIncMatx[i,j]
                if action !=0:
                    # print("%d * %d, " % (j+1, action), end="")
                    outputDist[i] = outputDist[i] + (self.droneZ[j]*-action)
        return outputDist

    def run(self):
        self.getPos()
        self.calcZmatX()
        XYDist = self.calcForm()
        return self.calcForm()
            # print(end='\r\n')
        # print(outputDist)
