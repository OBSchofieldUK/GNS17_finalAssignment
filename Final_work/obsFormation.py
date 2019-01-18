#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from math import sqrt
from scipy import linalg as la
import matplotlib.pyplot as pl

uavColours = ['r','g','b','k','y','c','m','w']

class formCtl(object):
    """docstring for formCtl."""
    def __init__(self, B, matDists, swarm, gain, inAlt):
        self.errLog = []
        self.desiredAlt = abs(inAlt)
        self.CoIncMatx = B
        self.desDistMatx = matDists
        self.velGain = gain
        self.numAgents, self.numEdges = self.CoIncMatx.shape
        self.droneSet = swarm
        self.dronePositions = np.zeros((1,self.numAgents*2))
        self.zDetails = []
        self.droneV = []
        self.atAltitude = False
        self.getPos()

    def calcZ(self, tailPos, headPos, edgeNum):
        Z = tailPos - headPos
        normZ = la.norm(Z)
        error = la.norm(Z) - self.desDistMatx[edgeNum]
        val = (Z/normZ)*error
        vel = 0.5 * (self.desDistMatx[edgeNum])**2
        self.zDetails.append([Z, error])
        return val , vel

    def getPos(self):   #gets all agent positions
        tmpPos = []
        for drone in self.droneSet:
            tmpPos.append(drone.xyz[0:2])
        self.dronePositions = np.concatenate(tmpPos)
        self.dronePositions = np.reshape(self.dronePositions, (self.numAgents,2))
        # print(self.dronePositions)

    def calcV(self): # calculate velocities (not used at this time)
        tmp = []
        for i in range(len(self.zDetails)):
            dat = self.zDetails[i]
            # print (la.norm(dat[0]))
            V = 0.5 * (dat[0] - self.desDistMatx[i])**2
            tmp.append(V)
        self.droneV = np.concatenate(tmp)
        self.droneV = np.reshape(self.droneV, (self.numEdges,2))
        # print(self.droneV)

    def calcZmatX(self):            # calculates all the Z distances in the coincidence matrix
        posTemp = []      # np.array((self.numEdges,2))
        velTemp = []
        pDot = np.array((1,self.numAgents))
        self.zDetails = []
        for j in range(self.numEdges):
            edgeTail = 0
            edgeHead = 0
            for i in range(self.numAgents):
                action = self.CoIncMatx[i,j]
                if action ==1:
                    edgeTail = i
                elif action == -1:
                    edgeHead = i
            position, velocity = self.calcZ(self.dronePositions[edgeTail], self.dronePositions[edgeHead],j)
            # print(position)
            posTemp.append(position)

        self.droneZ = np.concatenate(posTemp)
        self.droneZ = np.reshape(self.droneZ, (self.numEdges,2))
        # print(velTemp)
        # self.droneV = np.concatenate(velTemp)
        # self.droneV = np.reshape(self.droneV, (self.numEdges,2))

    def calcForm(self):             #Calculates the P Position X.Y
        outputDist = np.zeros((self.numAgents,2))
        outputVel = np.zeros_like(outputDist)
        for i in range(self.numAgents):
            # print(i+1, end='\t')
            posSum = 0
            velSum = 0

            for j in range(self.numEdges):
                action = self.CoIncMatx[i,j]
                if action !=0:
                    # print("%d * %d, " % (j+1, action), end="")
                    # outputDist[i] = outputDist[i] - (self.droneZ[j]*action)
                    posSum -= (self.droneZ[j]*action)
            outputDist[i] = posSum * (self.velGain)
            #print(end='\n')
        return outputDist

    def logError(self):
        tmp = []
        for i in range(len(self.zDetails)):
            data = self.zDetails[i]
            tmp.append(data[1])
        self.errLog.append(tmp)
        pass

    def moveToAlt(self, alt=0):
        self.getPos()
        if alt==0:
            alt=self.desiredAlt
        i = 0
        altMet = 0
        for uav in self.droneSet:
            xyz_D = np.array([self.dronePositions[i,0], self.dronePositions[i,1], -alt])
            uav.set_xyz_ned_lya(xyz_D)
            # print(i, uav.xyz, end='\t')
            i +=1
            if uav.xyz[2] <= -alt:
                altMet +=1

        # print(altMet)
        if altMet == len(self.droneSet):
            self.atAltitude=True
            return True
        else:
            self.atAltitude=False
            return False

    def run(self):
        self.getPos()
        # print()

        # print(self.moveToAlt(alt=0))
        if (self.moveToAlt(self.desiredAlt)):
            # print("entered")
            self.calcZmatX()
            XYDist = self.calcForm()
            for i in range(len(self.droneSet)):
                self.droneSet[i].set_v_2D_alt_lya([XYDist[i,0], XYDist[i,1]], alt_d=-self.desiredAlt)

            self.logError()


            # print(end='\r\n')
        # print(outputDist)

    def plotEdges(self):
        pl.figure(1)
        pl.clf()
        # for i in range(self.numAgents):
            # pl.plot(self.dronePositions[i], 'o'+uavColours[i])

        for j in range(self.numEdges):
            edgeTail = 0
            edgeHead = 0
            for i in range(self.numAgents):
                action = self.CoIncMatx[j,i]
                if action ==-1:
                    edgeTail = i
                elif action == 1:
                    edgeHead = i
                pl.plot(self.dronePositions[edgeTail],self.dronePositions[edgeHead],'r--',lw=2)
        pl.xlabel('South [m]')
        pl.ylabel('West [m]')
        pl.title('2D Map')
        pl.grid()
        pl.draw()
