import numpy as np
import pyrosim.pyrosim as pyrosim
from pyrosim.material import COLOR
import constants as c
import os
import random
import time
import copy


class SOLUTION:

    def __init__(self, solutionId):
        self.solutionId = solutionId
        self.numLinks = random.randint(2, 10)
        self.numSensors = random.randint(1, self.numLinks)
        self.weights = np.random.rand(self.numSensors * self.numLinks) * 2 - 1
        self.maxLinkSize = 2

        self.sensorArr = np.zeros(self.numLinks)
        for i in range(self.numSensors):
            randIndex = random.randint(0, self.numLinks - 1)
            if self.sensorArr[randIndex] == 1:
                i -= 1
            else:
                self.sensorArr[randIndex] = 1

    def CreateWorld(self):
        pyrosim.Start_SDF("world.sdf")

        pyrosim.Send_Cube(name='FirstStair', pos=[-52, 0, .125], size=[100, 100, .25])
        pyrosim.Send_Cube(name='SecondStair', pos=[-55, 0, .375], size=[100, 100, .25])
        pyrosim.Send_Cube(name='ThirdStair', pos=[-58, 0, .75], size=[100, 100, .5])
        pyrosim.Send_Cube(name='FourthStair', pos=[-61, 0, 1.25], size=[100, 100, .5])

        pyrosim.End()

    def MakeFirstLink(self, linkName):
        sizeArr = np.random.rand(3) * self.maxLinkSize
        posArr = np.array([0, 0, self.maxLinkSize / 2])
        if self.sensorArr[0] == 1:
            color = COLOR.Green
        else:
            color = COLOR.Cyan
        pyrosim.Send_Cube(name=linkName,
                          pos=[posArr[0], posArr[1], posArr[2]],
                          size=[sizeArr[0], sizeArr[1], sizeArr[2]],
                          color=color)
        self.links[linkName] = (linkName, posArr, sizeArr)
        self.lastKeyIndex = 0
        self.keyArr.append(linkName)

    def MakeLinkAndJoint(self, linkName, jointAxis):
        sizeArr = np.random.rand(3) * self.maxLinkSize
        lastLink = self.links[self.keyArr[self.lastKeyIndex]]

        posArr = copy.deepcopy(lastLink[1])
        jointPosArr = copy.deepcopy(lastLink[1])
        lastSizeArr = copy.deepcopy(lastLink[2])

        posArr[0] = sizeArr[0] / 2
        jointPosArr[0] = lastSizeArr[0]
        if self.lastKeyIndex == 0:
            jointPosArr[0] = lastSizeArr[0] / 2
            posArr[2] = 0

        if self.sensorArr[self.lastKeyIndex + 1] == 1:
            color = COLOR.Green
        else:
            color = COLOR.Cyan

        jointName = lastLink[0] + '_' + linkName
        pyrosim.Send_Joint(name=jointName,
                           parent=lastLink[0],
                           child=linkName,
                           type='revolute',
                           position=[jointPosArr[0], jointPosArr[1], jointPosArr[2]],
                           jointAxis=jointAxis)
        self.joints.append(jointName)
        pyrosim.Send_Cube(name=linkName,
                          pos=[posArr[0], posArr[1], posArr[2]],
                          size=[sizeArr[0], sizeArr[1], sizeArr[2]],
                          color=color)
        self.links[linkName] = (linkName, posArr, sizeArr)

        self.lastKeyIndex += 1
        self.keyArr.append(linkName)

    def GenerateBody(self):
        pyrosim.Start_URDF("body.urdf")

        self.links = dict()
        self.joints = list()
        self.keyArr = list()
        linkName = 0

        self.MakeFirstLink(str(linkName))
        linkName += 1

        for i in range(self.numLinks - 1):
            self.MakeLinkAndJoint(str(linkName), '0 1 0')
            linkName += 1

        pyrosim.End()

    def MakeSensorNeuron(self, link):
        pyrosim.Send_Sensor_Neuron(name=self.neuronId, linkName=link)
        self.sensors[link] = self.neuronId
        self.neuronId += 1

    def MakeMotorNeuron(self, joint):
        pyrosim.Send_Motor_Neuron(name=self.neuronId, jointName=joint)
        self.motors[joint] = self.neuronId
        self.neuronId += 1

    def MakeSynapse(self, sensorNeuron, motorNeuron, synapseIndex):
        sensor = self.sensors[sensorNeuron]
        motor = self.motors[motorNeuron]
        synapseWeight = self.weights[synapseIndex]
        pyrosim.Send_Synapse(sourceNeuronName=sensor,
                             targetNeuronName=motor,
                             weight=synapseWeight)

    def GenerateBrain(self):
        pyrosim.Start_NeuralNetwork("brain" + str(self.solutionId) + ".nndf")

        #int -> string
        self.sensors = dict()
        self.motors = dict()
        self.neuronId = 0

        for index in self.sensorArr:
            if index == 1:
                self.MakeSensorNeuron(self.keyArr[int(index)])

        for joint in self.joints:
            self.MakeMotorNeuron(joint)

        weightCounter = 0
        for sensor in self.sensors:
            for motor in self.motors:
                self.MakeSynapse(sensor, motor, weightCounter)
                weightCounter += 1

        pyrosim.End()

    def Wait_For_Simulation_To_End(self):
        fitnessFileName = "fitness" + str(self.solutionId) + ".txt"
        while not os.path.exists(fitnessFileName):
            time.sleep(0.01)
        f = open(fitnessFileName, "r")

        lines = f.readlines()
        xPos = lines[0]
        self.fitness = (float(xPos) * -1)

        # self.fitness = float(f.read()) * -1

        f.close()
        os.system("rm fitness" + str(self.solutionId) + ".txt")

    def Start_Simulation(self, connectionModeStr):
        # self.CreateWorld()
        self.GenerateBody()
        self.GenerateBrain()
        os.system("python simulate.py " + connectionModeStr + " " + str(self.solutionId) + " 2&>1 &")
        #os.system("python simulate.py " + connectionModeStr + " " + str(self.solutionId) + " &")

    def Mutate(self):
        randomIndex = random.randint(0, self.numSensors * self.numLinks - 1)
        self.weights[randomIndex] = random.random() * 2 - 1

    def Set_Id(self, newId):
        self.solutionId = newId

    def Extra(self):
        pass

