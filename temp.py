
backLegSensorValues = numpy.zeros(simulationLength)
frontLeg = numpy.zeros(simulationLength)
targetAnglesFront = numpy.linspace(0, 2 * numpy.pi, simulationLength)
targetAnglesBack = 



    targetAnglesFront[i] = 
    targetAnglesBack[i] = numpy.sin(c.frequency_back * targetAnglesBack[i] + c.offset_back) * c.amplitude_back

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
    