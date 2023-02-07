import pyrosim.pyrosim as pyrosim
import numpy as np
import constants as c

class SENSOR:

    def __init__(self, name):

        self.linkName = name
        self.values = np.zeros(c.simulationLength)
        self.sensorHistory = np.zeros(c.simulationLength)

        self.prevNumOfJumpIter = 0

    def Is_Oscillatory_Signal(self):
        return (self.linkName != "Head" or self.linkName != "Thorax")

    def Get_Value(self, loopIt):
        self.values[loopIt] = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)
        self.sensorHistory[loopIt] = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)

    def Get_Oscillation(self, loopIt):
        self.values[loopIt] = np.sin(c.gaitSpeed * loopIt)
        self.sensorHistory[loopIt] = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)

    def ComputeJumpTime(self, loopIt):
        if (loopIt != 0):
            if (self.sensorHistory[loopIt] == -1 and self.sensorHistory[(loopIt - 1)] == -1):
                self.prevNumOfJumpIter += 1
                return (self.prevNumOfJumpIter * (1/60))
            else:
                self.prevNumOfJumpIter = 0
                return 0
        return 0

    def Save_Values(self):
        np.save('data/sensor_values_' + self.linkName + '.txt', str(self.values))