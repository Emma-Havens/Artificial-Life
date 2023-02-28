import numpy as np
import random
import pyrosim.pyrosim as pyrosim
from pyrosim.material import COLOR
import copy
from link import LINK

class BODY:

    def __init__(self, numLinks, maxLinkSize, sensorArr):
        self.numLinks = numLinks
        self.maxLinkSize = maxLinkSize
        self.sensorArr = sensorArr
        self.links = dict()
        self.joints = list()
        self.keyArr = list()


    def GenerateBody(self):
        linkName = 0

        self.MakeFirstLink(str(linkName))
        linkName += 1

        for i in range(self.numLinks - 1):
            self.MakeLinkAndJoint(str(linkName), i + 1)
            linkName += 1

    def RegenerateBody(self, linkIndexToChange):
        self.joints = list()
        linktoChange = self.keyArr[linkIndexToChange]
        linkName = 0

        if linktoChange == str(linkName):
            self.MakeFirstLink(str(linkName))
        else:
            self.MakeFirstLink(str(linkName), True)

        linkName += 1

        for i in range(self.numLinks - 1):
            if linktoChange == str(linkName):
                self.MakeLinkAndJoint(str(linkName), i + 1)
            else:
                self.MakeLinkAndJoint(str(linkName), i + 1, True)
            linkName += 1

    def NoCollisions(self, potentialLink, curIndex):
        validLink = True
        print('attempting to place ' + potentialLink.linkName)
        for i in range(curIndex):
            link = self.keyArr[i]
            if (not potentialLink.NoCollision(self.links[link])):
                validLink = False
                print('collision with ' + str(link))
        return validLink

    def NoCollisions(self, potentialLink, curIndex):
        return True


    def MakeJointAxis(self, jointAxis):
        if (jointAxis == 0):
            return '1 0 0'
        elif (jointAxis == 1):
            return '0 1 0'
        elif (jointAxis == 2):
            return '0 0 1'

    def UpdateJointPosArr(self, lastSizeArr, lastPosArr, faceDir):
        if (faceDir == 1):
            return np.array([lastPosArr[0],
                             lastPosArr[1],
                             lastPosArr[2] + lastSizeArr[2] / 2])
        elif (faceDir == 2):
            return np.array([lastPosArr[0],
                             lastPosArr[1] + lastSizeArr[1] / -2,
                             lastPosArr[2]])
        elif (faceDir == 3):
            return np.array([lastPosArr[0],
                             lastPosArr[1],
                             lastPosArr[2] + lastSizeArr[2] / -2])
        elif (faceDir == 4):
            return np.array([lastPosArr[0],
                             lastPosArr[1] + lastSizeArr[1] / 2,
                             lastPosArr[2]])
        elif (faceDir == 5):
            return np.array([lastPosArr[0] + lastSizeArr[0] / 2,
                             lastPosArr[1],
                             lastPosArr[2]])

    def MakeFirstJoint(self, lastSizeArr, faceDir):
        if (faceDir == 1):
            return np.array([0,
                             0,
                             (lastSizeArr[2] / 2) + self.startingz])
        elif (faceDir == 2):
            return np.array([0,
                             lastSizeArr[1] / -2,
                             self.startingz])
        elif (faceDir == 3):
            return np.array([0,
                             0,
                             (lastSizeArr[2] / -2) + self.startingz])
        elif (faceDir == 4):
            return np.array([0,
                             lastSizeArr[1] / 2,
                             self.startingz])
        elif (faceDir == 5):
            return np.array([lastSizeArr[0] / 2,
                             0,
                             self.startingz])

    def UpdatePosArr(self, sizeArr, faceDir):
        if (faceDir == 1):
            return np.array([0,
                             0,
                             sizeArr[2] / 2])
        elif (faceDir == 2):
            return np.array([0,
                             sizeArr[1] / -2,
                             0])
        elif (faceDir == 3):
            return np.array([0,
                             0,
                             sizeArr[2] / -2])
        elif (faceDir == 4):
            return np.array([0,
                             sizeArr[1] / 2,
                             0])
        elif (faceDir == 5):
            return np.array([sizeArr[0] / 2,
                             0,
                             0])

    def MakeFirstLink(self, linkName, useExisting=False):
        if useExisting == True:
            sizeArr = self.links[self.keyArr[0]].sizeArr
        else:
            sizeArr = np.random.rand(3) * self.maxLinkSize
            self.keyArr.append(linkName)
            #sizeArr = np.ones(3)
        self.startingz = self.maxLinkSize * 2
        posArr = np.array([0, 0, self.startingz])
        if self.sensorArr[0] == 1:
            color = COLOR.Green
        else:
            color = COLOR.Cyan
        pyrosim.Send_Cube(name=linkName,
                          pos=[posArr[0], posArr[1], posArr[2]],
                          size=[sizeArr[0], sizeArr[1], sizeArr[2]],
                          color=color)
        self.links[linkName] = LINK(linkName, posArr, sizeArr, None, None, None, self.startingz)

    def MakeLinkAndJoint(self, linkName, thisIndex, useExisting=False):
        validLink = False

        lastLink = None
        sizeArr = None
        jointAxis = None
        posArr = None
        jointPosArr = None
        potentialLink = None
        while (not validLink):
            if useExisting == True:
                #print('existing true')
                thisLink = self.links[self.keyArr[thisIndex]]
                sizeArr = thisLink.sizeArr
                #print(sizeArr)
                jointAxis = thisLink.jointAxis
                faceDir = thisLink.faceDir
                lastLink = thisLink.prevLink
            else:
                #print('existing false')
                self.keyArr.append(linkName)
                sizeArr = np.random.rand(3) * self.maxLinkSize
                #sizeArr = np.ones(3)
                lastLink = self.links[self.keyArr[random.randint(0, thisIndex - 1)]]
                jointAxis = random.randint(0, 2)
                faceDir = random.randint(1, 5)
            #print(lastLink.linkName)

            lastSizeArr = copy.deepcopy(lastLink.sizeArr)

            jointPosArr = self.UpdateJointPosArr(lastSizeArr, lastLink.posArr, faceDir)
            posArr = self.UpdatePosArr(sizeArr, faceDir)
            if (lastLink.prevLink == None):
                jointPosArr = self.MakeFirstJoint(lastSizeArr, faceDir)

            potentialLink = LINK(linkName, posArr, sizeArr, jointAxis, lastLink, faceDir)
            if (self.NoCollisions(potentialLink, thisIndex)):
                validLink = True

        if self.sensorArr[thisIndex] == 1:
            color = COLOR.Green
        else:
            color = COLOR.Cyan

        jointName = lastLink.linkName + '_' + linkName
        #print(jointName)
        pyrosim.Send_Joint(name=jointName,
                           parent=lastLink.linkName,
                           child=linkName,
                           type='revolute',
                           position=[jointPosArr[0], jointPosArr[1], jointPosArr[2]],
                           jointAxis=self.MakeJointAxis(jointAxis))
        self.joints.append(jointName)
        pyrosim.Send_Cube(name=linkName,
                          pos=[posArr[0], posArr[1], posArr[2]],
                          size=[sizeArr[0], sizeArr[1], sizeArr[2]],
                          color=color)
        self.links[linkName] = potentialLink

