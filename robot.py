import pybullet as p
import pyrosim.pyrosim as pyrosim

from pyrosim.neuralNetwork import NEURAL_NETWORK
from sensor import SENSOR
from motor import MOTOR

class ROBOT:

    def __init__(self):
        self.sensors = dict()
        self.motors = dict()

        self.robotId = p.loadURDF("body.urdf")
        self.nn = NEURAL_NETWORK("brain.nndf")

        pyrosim.Prepare_To_Simulate(self.robotId)
        self.Prepare_To_Sense()
        self.Prepare_To_Act()

    def Prepare_To_Sense(self): 
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

    def Sense(self, loopIt):
        for sensor_i in self.sensors:
            self.sensors[sensor_i].Get_Value(loopIt)

    def Prepare_To_Act(self):
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName)
            if (jointName == 'Torso_BackLeg'):
                self.motors[jointName].frequency = self.motors[jointName].frequency / 2

    def Act(self, loopIt):
        for joint_i in self.motors:
            motorVal = self.motors[joint_i].Set_Value(loopIt)
            pyrosim.Set_Motor_For_Joint(
                    bodyIndex = self.robotId,
                    jointName = self.motors[joint_i].jointName,
                    controlMode = p.POSITION_CONTROL,
                    targetPosition = self.motors[joint_i].values[loopIt],
                    maxForce = 15)

    def Think(self):
        self.nn.Print()

    def Save_Values(self):
        for i in self.sensors:
            self.sensors[i].Save_Values()
        for i in self.motors:
            self.motors[i].Save_Values()