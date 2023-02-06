import pyrosim.pyrosim as pyrosim
import numpy as np
import constants as c

class SENSOR:

    def __init__(self, name):

        self.linkName = name
        self.values = np.zeros(c.simulationLength)

    def Get_Value(self, loopIt):
        #self.values[loopIt] = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)
        self.values[loopIt] = np.sin(c.gaitSpeed * loopIt)

    def Save_Values(self):
        np.save('data/sensor_values_' + self.linkName, self.values)