import numpy as np
import pyrosim.pyrosim as pyrosim
import constants as c
import os
import random
import time

class SOLUTION:

    def __init__(self, solutionId):
        self.solutionId = solutionId
        self.weights = np.random.rand(c.numSensorNeurons, c.numMotorNeurons) * 2 - 1

    def CreateWorld(self):
        pyrosim.Start_SDF("world.sdf")
        pyrosim.Send_Cube(name='Box', pos=[4, 5, .5] , size=[1, 1, 1])
        pyrosim.End()

    def GenerateBody(self):
        pyrosim.Start_URDF("body.urdf")

        pyrosim.Send_Cube(name='Torso', pos=[0, 0, 1] , size=[1, 1, 1])

        #Front Leg
        pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" ,
                            type = "revolute", position = [0, .5, 1], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name='FrontLeg', pos=[0, .5, 0] , size=[.2, 1, .2])
        pyrosim.Send_Joint( name = "FrontLeg_FrontLowerLeg" , parent= "FrontLeg" , child = "FrontLowerLeg" ,
                            type = "revolute", position = [0, 1, 0], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name='FrontLowerLeg', pos=[0, 0, -.5] , size=[.2, .2, 1])

        #Back Leg
        pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" ,
                            type = "revolute", position = [0, -.5, 1], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name='BackLeg', pos=[0, -.5, 0] , size=[.2, 1, .2])
        pyrosim.Send_Joint( name = "BackLeg_BackLowerLeg" , parent= "BackLeg" , child = "BackLowerLeg" ,
                            type = "revolute", position = [0, -1, 0], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name='BackLowerLeg', pos=[0, 0, -.5] , size=[.2, .2, 1])

        #Left Leg
        pyrosim.Send_Joint( name = "Torso_LeftLeg" , parent= "Torso" , child = "LeftLeg" ,
                            type = "revolute", position = [-.5, 0, 1], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name='LeftLeg', pos=[-.5, 0, 0] , size=[1, .2, .2])
        pyrosim.Send_Joint( name = "LeftLeg_LeftLowerLeg" , parent= "LeftLeg" , child = "LeftLowerLeg" ,
                            type = "revolute", position = [-1, 0, 0], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name='LeftLowerLeg', pos=[0, 0, -.5] , size=[.2, .2, 1])

        #Right Leg
        pyrosim.Send_Joint( name = "Torso_RightLeg" , parent= "Torso" , child = "RightLeg" ,
                            type = "revolute", position = [.5, 0, 1], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name='RightLeg', pos=[.5, 0, 0] , size=[1, .2, .2])
        pyrosim.Send_Joint( name = "RightLeg_RightLowerLeg" , parent= "RightLeg" , child = "RightLowerLeg" ,
                            type = "revolute", position = [1, 0, 0], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name='RightLowerLeg', pos=[0, 0, -.5] , size=[.2, .2, 1])

        pyrosim.End()

    def MakeSensorNeuron(self, idName, link):
        pyrosim.Send_Sensor_Neuron(name = idName, linkName = link)
        self.sensorNeurons.append(idName)     

    def MakeMotorNeuron(self, idName, joint):
        pyrosim.Send_Motor_Neuron(name = idName, jointName = joint)
        self.motorNeurons.append(idName)

    def GenerateBrain(self):
        pyrosim.Start_NeuralNetwork("brain" + str(self.solutionId) + ".nndf")

        self.sensorNeurons = []
        self.motorNeurons = []

        self.MakeSensorNeuron(0, "BackLowerLeg")
        self.MakeSensorNeuron(1, "FrontLowerLeg")
        self.MakeSensorNeuron(2, "LeftLowerLeg")
        self.MakeSensorNeuron(3, "RightLowerLeg")

        self.MakeMotorNeuron(4, "BackLeg_BackLowerLeg")
        self.MakeMotorNeuron(5, "FrontLeg_FrontLowerLeg")
        self.MakeMotorNeuron(6, "LeftLeg_LeftLowerLeg")
        self.MakeMotorNeuron(7, "RightLeg_RightLowerLeg")

        for sensor in self.sensorNeurons:
            for i, motor in enumerate(self.motorNeurons):
                pyrosim.Send_Synapse( sourceNeuronName = sensor,
                                      targetNeuronName = motor,
                                      weight = self.weights[sensor][i] )

        pyrosim.End()

    def Wait_For_Simulation_To_End(self):
        fitnessFileName = "fitness" + str(self.solutionId) + ".txt"
        while not os.path.exists(fitnessFileName):
            time.sleep(0.01)
        f = open(fitnessFileName, "r")
        self.fitness = float(f.read())
        f.close()
        os.system("rm fitness" + str(self.solutionId) + ".txt")

    def Start_Simulation(self, connectionModeStr):
        self.CreateWorld()
        self.GenerateBody()
        self.GenerateBrain()
        os.system("python simulate.py " + connectionModeStr + " " + str(self.solutionId) + " 2&>1 &")

    def Mutate(self):
        randomRow = random.randint(0, 2)
        randomCol = random.randint(0, 1)
        self.weights[randomRow][randomCol] = random.random() * 2 - 1

    def Set_Id(self, newId):
        self.solutionId = newId