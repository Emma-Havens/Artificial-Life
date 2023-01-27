import numpy as np
import pyrosim.pyrosim as pyrosim
import os
import random

class SOLUTION:

    def __init__(self):
        self.weights = np.random.rand(3, 2) * 2 - 1

    def CreateWorld(self):
        pyrosim.Start_SDF("world.sdf")
        pyrosim.Send_Cube(name='Box', pos=[4, 5, .5] , size=[1, 1, 1])
        pyrosim.End()

    def GenerateBody(self):
        pyrosim.Start_URDF("body.urdf")
        pyrosim.Send_Cube(name='Torso', pos=[1.5, 0, 1.5] , size=[1, 1, 1])
        pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [1, 0, 1])
        pyrosim.Send_Cube(name='FrontLeg', pos=[-.5, 0, -.5] , size=[1, 1, 1])
        pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [2, 0, 1])
        pyrosim.Send_Cube(name='BackLeg', pos=[.5, 0, -.5] , size=[1, 1, 1])
        pyrosim.End()

    def MakeSensorNeuron(self, idName, link):
        pyrosim.Send_Sensor_Neuron(name = idName, linkName = link)
        self.sensorNeurons.append(idName)     

    def MakeMotorNeuron(self, idName, joint):
        pyrosim.Send_Motor_Neuron(name = idName, jointName = joint)
        self.motorNeurons.append(idName)

    def GenerateBrain(self):
        pyrosim.Start_NeuralNetwork("brain.nndf")

        self.sensorNeurons = []
        self.motorNeurons = []

        self.MakeSensorNeuron(0, "Torso")
        self.MakeSensorNeuron(1, "BackLeg")
        self.MakeSensorNeuron(2, "FrontLeg")

        self.MakeMotorNeuron(3, "Torso_BackLeg")
        self.MakeMotorNeuron(4, "Torso_FrontLeg")

        for sensor in self.sensorNeurons:
            for i, motor in enumerate(self.motorNeurons):
                pyrosim.Send_Synapse( sourceNeuronName = sensor,
                                      targetNeuronName = motor,
                                      weight = self.weights[sensor][i] )

        pyrosim.End()

    def Evaluate(self, connectionModeStr):
        self.CreateWorld()
        self.GenerateBody()
        self.GenerateBrain()
        os.system("python simulate.py " + connectionModeStr)

        f = open("fitness.txt", "r")
        self.fitness = float(f.read())
        f.close()

    def Mutate(self):
        randomRow = random.randint(0, 2)
        randomCol = random.randint(0, 1)
        self.weights[randomRow][randomCol] = random.random() * 2 - 1