import pybullet as p
import pybullet_data
import time

import constants as c
from robot import ROBOT
from world import WORLD

class SIMULATION:

    def __init__(self):

        physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        p.setGravity(0,0,-9.8)

        self.world = WORLD()
        self.robot = ROBOT()

    def Run(self):
        for i in range(c.simulationLength):
            #print(i)
            time.sleep(1/60)
            p.stepSimulation()

            self.robot.Sense(i)
            self.robot.Act(i)

        for sensor_i in self.robot.sensors:
            #print(self.robot.sensors[sensor_i].values)
            pass

        for joint_i in self.robot.motors:
            #print(self.robot.motors[joint_i].values)
            pass

    def __del__(self):
        p.disconnect()
