import pyrosim.pyrosim as pyrosim

pyrosim.Start_SDF("boxes.sdf")

length = 1
width = 1
height = 1

x = 0
y = 0
z = .5

for r in range(5):
    for c in range(5):
        for i in range(10):
            boxName = 'Box' + str(i)
            pyrosim.Send_Cube(name=boxName, pos=[x, y, z] , size=[width, length, height])
            length = length * .9
            width = width * .9
            height = height * .9
            z += 1
        length = 1
        width = 1
        height = 1
        z = .5
        y += 1
    y = 0
    x += 1    

pyrosim.End()