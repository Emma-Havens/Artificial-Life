import numpy as np
import matplotlib.pyplot as matplot

backLegSensorValues = np.load('data/grounded.npy')
frontLegSensorValues = np.load('data/front_grounded.npy')
targetAngleFront = np.load('data/sinusoidal_front.npy')
targetAngleBack = np.load('data/sinusoidal_back.npy')
print(backLegSensorValues)

matplot.plot(backLegSensorValues, label='back leg', linewidth=4)
matplot.plot(frontLegSensorValues, label='front leg')

matplot.legend()
matplot.show()

matplot.plot(targetAngleFront)
matplot.plot(targetAngleBack)
matplot.show()

