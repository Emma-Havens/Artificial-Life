import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import sys

import numpy
import random

import constants as c
from simulation import SIMULATION

DirectOrGUI = sys.argv[1]
solutionId = sys.argv[2]
sim = SIMULATION(DirectOrGUI, solutionId)
sim.Run()
sim.GetFitness()