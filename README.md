# Artificial-Life

Simulated using pybullet, robot resources from pyrosim. See r/ludobots for an in depth explanation.

Project can easily be run a reproduced by running 'python search.py'

Project creates a chain of links, randomly sized between 0 and 3 units in any dimension. There are a random number of links, between 2 and 10. All of these constants can be easily changed, but were selected for both breadth and clarity. A random number of sensors (less than the random number of links) are placed at randomly selected positions and are indicated by green colored links. All links without sensors are colored cyan. Links have joints between them that allow the blocks to move up and down respective to one another. All joints are motorized, and all sensors send inputs to all motors via synapses.
