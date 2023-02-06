import numpy as np
import pyrosim.pyrosim as pyrosim
import constants as c
import os
import random
import time

class SOLUTION:

    def __init__(self, solutionId):
        self.solutionId = solutionId
        self.weights = np.random.rand(c.numSynapse) * 2 - 1

    def CreateWorld(self):
        pyrosim.Start_SDF("world.sdf")
        pyrosim.Send_Cube(name='Box', pos=[4, 5, .5] , size=[1, 1, 1])
        pyrosim.End()

    def GenerateBody(self):
        pyrosim.Start_URDF("body.urdf")

        #Body
        pyrosim.Send_Cube(name='Head', pos=[-.25, 0, 1] , size=[1.5, 1, 1])
        pyrosim.Send_Joint( name = "Head_Thorax" , parent= "Head" , child = "Thorax" ,
                            type = "revolute", position = [.5, 0, 1], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name='Thorax', pos=[.5, 0, 0] , size=[1, 1, 1])
        pyrosim.Send_Joint( name = "Thorax_Abdomen" , parent= "Thorax" , child = "Abdomen" ,
                            type = "revolute", position = [1, 0, 0], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name='Abdomen', pos=[.75, 0, 0] , size=[1.5, 1, 1])

        #Front Left Leg
        pyrosim.Send_Joint( name = "Head_FrontLeft" , parent= "Head" , child = "FrontLeft" ,
                            type = "revolute", position = [0, -.5, 1], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name='FrontLeft', pos=[0, -.5, 0] , size=[.2, 1, .2])
        pyrosim.Send_Joint( name = "FrontLeft_FrontLeftLower" , parent= "FrontLeft" , child = "FrontLeftLower" ,
                            type = "revolute", position = [0, -1, 0], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name='FrontLeftLower', pos=[0, 0, -.5] , size=[.2, .2, 1])

        #Front Right Leg
        pyrosim.Send_Joint( name = "Head_FrontRight" , parent= "Head" , child = "FrontRight" ,
                            type = "revolute", position = [0, .5, 1], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name='FrontRight', pos=[0, .5, 0] , size=[.2, 1, .2])
        pyrosim.Send_Joint( name = "FrontRight_FrontRightLower" , parent= "FrontRight" , child = "FrontRightLower" ,
                            type = "revolute", position = [0, 1, 0], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name='FrontRightLower', pos=[0, 0, -.5] , size=[.2, .2, 1])

        #Mid Left Leg
        pyrosim.Send_Joint( name = "Thorax_MidLeft" , parent= "Thorax" , child = "MidLeft" ,
                            type = "revolute", position = [.5, -.5, 0], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name='MidLeft', pos=[0, -.5, 0] , size=[.2, 1, .2])
        pyrosim.Send_Joint( name = "MidLeft_MidLeftLower" , parent= "MidLeft" , child = "MidLeftLower" ,
                            type = "revolute", position = [0, -1, 0], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name='MidLeftLower', pos=[0, 0, -.5] , size=[.2, .2, 1])

        #Mid Right Leg
        pyrosim.Send_Joint( name = "Thorax_MidRight" , parent= "Thorax" , child = "MidRight" ,
                            type = "revolute", position = [.5, .5, 0], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name='MidRight', pos=[0, .5, 0] , size=[.2, 1, .2])
        pyrosim.Send_Joint( name = "MidRight_MidRightLower" , parent= "MidRight" , child = "MidRightLower" ,
                            type = "revolute", position = [0, 1, 0], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name='MidRightLower', pos=[0, 0, -.5] , size=[.2, .2, 1])

        #Back Left Leg
        pyrosim.Send_Joint( name = "Abdomen_BackLeft" , parent= "Abdomen" , child = "BackLeft" ,
                            type = "revolute", position = [.5, -.5, 0], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name='BackLeft', pos=[0, -.5, 0] , size=[.2, 1, .2])
        pyrosim.Send_Joint( name = "BackLeft_BackLeftLower" , parent= "BackLeft" , child = "BackLeftLower" ,
                            type = "revolute", position = [0, -1, 0], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name='BackLeftLower', pos=[0, 0, -.5] , size=[.2, .2, 1])

        #Back Right Leg
        pyrosim.Send_Joint( name = "Abdomen_BackRight" , parent= "Abdomen" , child = "BackRight" ,
                            type = "revolute", position = [.5, .5, 0], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name='BackRight', pos=[0, .5, 0] , size=[.2, 1, .2])
        pyrosim.Send_Joint( name = "BackRight_BackRightLower" , parent= "BackRight" , child = "BackRightLower" ,
                            type = "revolute", position = [0, 1, 0], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name='BackRightLower', pos=[0, 0, -.5] , size=[.2, .2, 1])

        pyrosim.End()

    def MakeSensorNeuron(self, dictKey, link):
        pyrosim.Send_Sensor_Neuron(name = self.neuronId, linkName = link)
        self.sensorNeurons[dictKey] = self.neuronId
        self.neuronId += 1     

    def MakeMotorNeuron(self, dictKey, joint):
        pyrosim.Send_Motor_Neuron(name = self.neuronId, jointName = joint)
        self.motorNeurons[dictKey] = self.neuronId
        self.neuronId += 1

    def MakeSynapse(self, sensorNeuron, motorNeuron, synapseIndex):
        sensor = self.sensorNeurons[sensorNeuron]
        motor = self.motorNeurons[motorNeuron]
        synapseWeight = self.weights[synapseIndex]
        pyrosim.Send_Synapse( sourceNeuronName = sensor,
                              targetNeuronName = motor,
                              weight = synapseWeight )
        self.nextSynapseWeightIndex += 1

    def GenerateBrain(self):
        pyrosim.Start_NeuralNetwork("brain" + str(self.solutionId) + ".nndf")

        self.sensorNeurons = dict()
        self.motorNeurons = dict()
        self.neuronId = 0

        self.MakeSensorNeuron('H', "Head")
        self.MakeSensorNeuron('FLL', "FrontLeftLower")
        self.MakeSensorNeuron('FRL', "FrontRightLower")
        self.MakeSensorNeuron('MLL', "MidLeftLower")
        self.MakeSensorNeuron('MRL', "MidRightLower")
        self.MakeSensorNeuron('BLL', "BackLeftLower")
        self.MakeSensorNeuron('BRL', "BackRightLower")

        self.MakeMotorNeuron('H_T', "Head_Thorax")
        self.MakeMotorNeuron('T_A', "Thorax_Abdomen")
        self.MakeMotorNeuron('H_FL', "Head_FrontLeft")
        self.MakeMotorNeuron('H_FR', "Head_FrontRight")
        self.MakeMotorNeuron('FL_FLL', "FrontLeft_FrontLeftLower")
        self.MakeMotorNeuron('FR_FRL', "FrontRight_FrontRightLower")
        self.MakeMotorNeuron('T_ML', "Thorax_MidLeft")
        self.MakeMotorNeuron('T_MR', "Thorax_MidRight")
        self.MakeMotorNeuron('ML_MLL', "MidLeft_MidLeftLower")
        self.MakeMotorNeuron('MR_MRL', "MidRight_MidRightLower")
        self.MakeMotorNeuron('A_BL', "Abdomen_BackLeft")
        self.MakeMotorNeuron('A_BR', "Abdomen_BackRight")
        self.MakeMotorNeuron('BL_BLL', "BackLeft_BackLeftLower")
        self.MakeMotorNeuron('BR_BRL', "BackRight_BackRightLower")

        self.nextSynapseWeightIndex = 0

        #self.MakeSynapse('H', 'H_T', 0)
        #self.MakeSynapse('H', 'T_A', 1)

        self.MakeSynapse('FLL', 'FL_FLL', 2)
        self.MakeSynapse('FLL', 'MR_MRL', 2)
        self.MakeSynapse('FLL', 'BL_BLL', 2)
        self.MakeSynapse('MRL', 'FL_FLL', 2)
        self.MakeSynapse('MRL', 'MR_MRL', 2)
        self.MakeSynapse('MRL', 'BL_BLL', 2)
        self.MakeSynapse('BLL', 'FL_FLL', 2)
        self.MakeSynapse('BLL', 'MR_MRL', 2)
        self.MakeSynapse('BLL', 'BL_BLL', 2)


        self.MakeSynapse('FRL', 'FR_FRL', 3)
        self.MakeSynapse('FRL', 'ML_MLL', 3)
        self.MakeSynapse('FRL', 'BR_BRL', 3)
        self.MakeSynapse('MLL', 'FR_FRL', 3)
        self.MakeSynapse('MLL', 'ML_MLL', 3)
        self.MakeSynapse('MLL', 'BR_BRL', 3)
        self.MakeSynapse('BRL', 'FR_FRL', 3)
        self.MakeSynapse('BRL', 'ML_MLL', 3)
        self.MakeSynapse('BRL', 'BR_BRL', 3)


        self.MakeSynapse('FLL', 'H_FL', 0)
        self.MakeSynapse('FLL', 'T_MR', 0)
        self.MakeSynapse('FLL', 'A_BL', 0)
        self.MakeSynapse('MRL', 'H_FL', 0)
        self.MakeSynapse('MRL', 'T_MR', 0)
        self.MakeSynapse('MRL', 'A_BL', 0)
        self.MakeSynapse('BLL', 'H_FL', 0)
        self.MakeSynapse('BLL', 'T_MR', 0)
        self.MakeSynapse('BLL', 'A_BL', 0)


        self.MakeSynapse('FRL', 'H_FR', 1)
        self.MakeSynapse('FRL', 'T_ML', 1)
        self.MakeSynapse('FRL', 'A_BR', 1)
        self.MakeSynapse('MLL', 'H_FR', 1)
        self.MakeSynapse('MLL', 'T_ML', 1)
        self.MakeSynapse('MLL', 'A_BR', 1)
        self.MakeSynapse('BRL', 'H_FR', 1)
        self.MakeSynapse('BRL', 'T_ML', 1)
        self.MakeSynapse('BRL', 'A_BR', 1)
        

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
        randomIndex = random.randint(0, c.numSynapse - 1)
        self.weights[randomIndex] = random.random() * 2 - 1

    def Set_Id(self, newId):
        self.solutionId = newId

    def Extra(self):
        pass