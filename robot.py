import pybullet as p
import pyrosim.pyrosim as pyrosim
import os
import numpy as np
import constants as c

from pyrosim.neuralNetwork import NEURAL_NETWORK
from sensor import SENSOR
from motor import MOTOR

class ROBOT:

    def __init__(self, solutionId):
        self.sensors = dict()
        self.motors = dict()

        self.solutionId = solutionId
        self.robotId = p.loadURDF("body.urdf")
        self.nn = NEURAL_NETWORK("brain" + str(self.solutionId) + ".nndf")
        os.system("rm brain" + str(self.solutionId) + ".nndf")

        pyrosim.Prepare_To_Simulate(self.robotId)
        self.Prepare_To_Sense()

        self.LegSensors = ["FrontLeftLower", "FrontRightLower", "MidLeftLower", "MidRightLower"]
        self.JumpTime = [ 0, 0, 0, 0]

    def Prepare_To_Sense(self): 
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

    def Sense(self, loopIt):
        for sensor_i in self.sensors:
            if (self.sensors[sensor_i].Is_Oscillatory_Signal()):
                self.sensors[sensor_i].Get_Oscillation(loopIt)
            else:
                self.sensors[sensor_i].Get_Value(loopIt)

        for i, leg in enumerate(self.LegSensors):
            jump = self.sensors[leg].ComputeJumpTime(loopIt)
            if (jump > self.JumpTime[i]):
                self.JumpTime[i] = jump                


    def Act(self, loopIt):
        for neuronName in self.nn.Get_Neuron_Names():

            if self.nn.Is_Motor_Neuron(neuronName):
                jointId = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = self.nn.Get_Value_Of(neuronName) * c.motorJointRange
                #print(neuronName + ", " + str(desiredAngle))

                pyrosim.Set_Motor_For_Joint(
                    bodyIndex = self.robotId,
                    jointName = jointId,
                    controlMode = p.POSITION_CONTROL,
                    targetPosition = desiredAngle,
                    maxForce = 50)            

    def Think(self):
        self.nn.Update()
        #self.nn.Print()

    def Get_Fitness(self):
        basePositionAndOrientation = p.getBasePositionAndOrientation(self.robotId)
        basePosition = basePositionAndOrientation[0]
        xPosition = basePosition[0]
        zPosition = basePosition[2]

        meanJumpTime = np.sum(self.JumpTime) / 4

        headOffGround = np.sum(self.sensors["Head"].sensorHistory) / self.sensors["Head"].sensorHistory.size

        f = open("tmp" + str(self.solutionId) + ".txt", "w")
        f.write(str(xPosition) + "\n" + str(zPosition) + "\n" + str(meanJumpTime) + "\n" + str(headOffGround))
        f.close()
        os.system("mv tmp" + str(self.solutionId) + ".txt fitness" + str(self.solutionId) + ".txt")

    def Save_Values(self):
        for i in self.sensors:
            self.sensors[i].Save_Values()
        #for i in self.motors:
            #self.motors[i].Save_Values()