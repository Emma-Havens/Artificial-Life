import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import sys

import numpy
import random

import constants as c
from simulation import SIMULATION

DirectOrGUI = sys.argv[1]
sim = SIMULATION(DirectOrGUI)
sim.Run()
sim.GetFitness()