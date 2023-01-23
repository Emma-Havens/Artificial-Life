import pyrosim.pyrosim as pyrosim
import random

length = 1
width = 1
height = 1

sensorNeurons = []
motorNeurons = []

def CreateWorld():
    pyrosim.Start_SDF("world.sdf")
    pyrosim.Send_Cube(name='Box', pos=[4, 5, .5] , size=[width, length, height])
    pyrosim.End()

def Generate_Body():
    pyrosim.Start_URDF("body.urdf")
    pyrosim.Send_Cube(name='Torso', pos=[1.5, 0, 1.5] , size=[width, length, height])
    pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [1, 0, 1])
    pyrosim.Send_Cube(name='FrontLeg', pos=[-.5, 0, -.5] , size=[width, length, height])
    pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [2, 0, 1])
    pyrosim.Send_Cube(name='BackLeg', pos=[.5, 0, -.5] , size=[width, length, height])
    pyrosim.End()

def MakeSensorNeuron(idName, link):
    pyrosim.Send_Sensor_Neuron(name = idName, linkName = link)
    sensorNeurons.append(idName)     

def MakeMotorNeuron(idName, joint):
    pyrosim.Send_Motor_Neuron(name = idName, jointName = joint)
    motorNeurons.append(idName)

def Generate_Brain():
    pyrosim.Start_NeuralNetwork("brain.nndf")

    MakeSensorNeuron(0, "Torso")
    MakeSensorNeuron(1, "BackLeg")
    MakeSensorNeuron(2, "FrontLeg")

    MakeMotorNeuron(3, "Torso_BackLeg")
    MakeMotorNeuron(4, "Torso_FrontLeg")

    for sensor in sensorNeurons:
        for motor in motorNeurons:
            pyrosim.Send_Synapse( sourceNeuronName = sensor,
                                  targetNeuronName = motor,
                                  weight = random.uniform(-1, 1) )

    pyrosim.End()

CreateWorld()
Generate_Body()
Generate_Brain()