import numpy as np
import pyrosim.pyrosim as pyrosim

import constants as c
import os
import random
import time

from body import BODY


class SOLUTION:

    def __init__(self, solutionId):
        self.solutionId = solutionId
        self.body = None
        self.numLinks = random.randint(3, 4)
        self.numSensors = random.randint(1, self.numLinks)
        self.weights = np.random.rand(self.numSensors * self.numLinks) * 2 - 1
        self.maxLinkSize = 1

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

    def GenerateBody(self):
        pyrosim.Start_URDF("body" + str(self.solutionId) + ".urdf")

        self.body = BODY(self.numLinks, self.maxLinkSize, self.sensorArr)
        self.body.GenerateBody()

        pyrosim.End()

    def RegenerateChildBody(self, randLink):
        pyrosim.Start_URDF("body" + str(self.solutionId) + ".urdf")

        self.body.RegenerateBody(randLink)

        pyrosim.End()

    def printBodySize(self):
        for link in self.body.links:
            print(self.body.links[link].sizeArr)

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
                self.MakeSensorNeuron(self.body.keyArr[int(index)])

        for joint in self.body.joints:
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
        dist = lines[0]
        self.fitness = (float(dist))

        # self.fitness = float(f.read()) * -1

        f.close()
        os.system("rm fitness" + str(self.solutionId) + ".txt")

    def Start_Simulation(self, connectionModeStr):
        # self.CreateWorld()
        if self.body == None:
            self.GenerateBody()
        self.GenerateBrain()
        os.system("python simulate.py " + connectionModeStr + " " + str(self.solutionId) + " 2&>1 &")
        #os.system("python simulate.py " + connectionModeStr + " " + str(self.solutionId) + " &")

    def Mutate(self):
        #randomIndex = random.randint(0, self.numSensors * self.numLinks - 1)
        #self.weights[randomIndex] = random.random() * 2 - 1

        randomLink = random.randint(0, self.numLinks - 1)
        self.RegenerateChildBody(randomLink)


    def Set_Id(self, newId):
        self.solutionId = newId

    def Extra(self):
        pass

