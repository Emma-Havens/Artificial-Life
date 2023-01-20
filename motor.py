import numpy as np
import constants as c

class MOTOR:

    def __init__(self, name):

        self.jointName = name
        self.values = np.linspace(0, 2 * np.pi, c.simulationLength)

        self.frequency = 7
        self.offset = 0
        self.amplitude = np.pi / 4

    def Set_Value(self, loopIt):
        self.values[loopIt] = np.sin(self.frequency * self.values[loopIt] + self.offset) * self.amplitude

    def Save_Values(self):
        np.save('data/motor_values_' + self.jointName, self.values)