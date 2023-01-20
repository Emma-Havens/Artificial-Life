import pyrosim.pyrosim as pyrosim

length = 1
width = 1
height = 1

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

def Generate_Brain():
    pyrosim.Start_NeuralNetwork("brain.nndf")
    pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
    pyrosim.End()

CreateWorld()
Generate_Body()
Generate_Brain()