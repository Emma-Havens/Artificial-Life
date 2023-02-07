﻿import pybullet as p
import pybullet_data
import time
import logging

import constants as c
from robot import ROBOT
from world import WORLD

class SIMULATION:

    def __init__(self, connectionMode, solutionId):

        self.connectionMode = connectionMode

        if (self.connectionMode == "DIRECT"):
            physicsClient = p.connect(p.DIRECT)
        else:
            physicsClient = p.connect(p.GUI)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        p.setGravity(0,0,-9.8)

        self.world = WORLD()
        self.robot = ROBOT(solutionId)

    def Run(self):
        for i in range(c.simulationLength):

            if (self.connectionMode == "GUI"):
                time.sleep(1/60)
            p.stepSimulation()

            self.robot.Sense(i)
            self.robot.Think()
            self.robot.Act(i)

        for sensor_i in self.robot.sensors:
            #print(self.robot.sensors[sensor_i].values)
            pass

        for joint_i in self.robot.motors:
            #print(self.robot.motors[joint_i].values)
            pass

    def GetFitness(self):
        self.robot.Get_Fitness()

    def Save_Values(self):
        self.robot.Save_Values()

    def __del__(self):
        p.disconnect()
