import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import time
import numpy

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")

p.loadSDF("world.sdf")
pyrosim.Prepare_To_Simulate(robotId)

backLegSensorValues = numpy.zeros(100)
frontLeg = numpy.zeros(100)

for i in range(100):
    time.sleep(1/60)
    p.stepSimulation()
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    frontLeg[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
    print(backLegSensorValues[i])

numpy.save('data/grounded.npy', backLegSensorValues)
numpy.save('data/front_grounded.npy', frontLeg)
    

p.disconnect()