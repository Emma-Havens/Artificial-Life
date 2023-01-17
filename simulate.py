import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import time
import numpy
import random

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")

p.loadSDF("world.sdf")
pyrosim.Prepare_To_Simulate(robotId)

backLegSensorValues = numpy.zeros(1000)
frontLeg = numpy.zeros(1000)
targetAnglesFront = numpy.linspace(0, 2 * numpy.pi, 1000)
targetAnglesBack = numpy.linspace(0, 2 * numpy.pi, 1000)

amplitudefront = numpy.pi / 4
frequencyfront = -.5
offsetfront = 0

amplitudeback = numpy.pi / 3
frequencyback = 20
offsetback = 0

for i in range(1000):
    time.sleep(1/60)
    p.stepSimulation()

    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    frontLeg[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
    targetAnglesFront[i] = numpy.sin(frequencyfront * targetAnglesFront[i] + offsetfront) * amplitudefront
    targetAnglesBack[i] = numpy.sin(frequencyback * targetAnglesBack[i] + offsetback) * amplitudeback

    print(i)

    pyrosim.Set_Motor_For_Joint(
        bodyIndex = robotId,
        jointName = "Torso_BackLeg",
        controlMode = p.POSITION_CONTROL,
        targetPosition = targetAnglesBack[i],
        maxForce = 15)
    pyrosim.Set_Motor_For_Joint(
        bodyIndex = robotId,
        jointName = "Torso_FrontLeg",
        controlMode = p.POSITION_CONTROL,
        targetPosition = targetAnglesFront[i],
        maxForce = 15)

numpy.save('data/grounded.npy', backLegSensorValues)
numpy.save('data/front_grounded.npy', frontLeg)
numpy.save('data/sinusoidal_back.npy', targetAnglesBack)
numpy.save('data/sinusoidal_front.npy', targetAnglesFront)
    

p.disconnect()