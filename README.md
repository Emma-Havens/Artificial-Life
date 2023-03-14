# Artificial-Life
![TeaserGIF](https://user-images.githubusercontent.com/71985604/224880877-93e3ca6d-baf8-4f5d-870d-43a55d96cda8.gif)

Simulated using pybullet, robot resources from pyrosim. See r/ludobots for an in depth explanation.

Project can easily be run a reproduced by running 'python search.py'
To view results of reproduction, run 'python review_pickle.py'
To see some pickles from me, see above. You need the .urdf files as well.

**Links:** A 3D block, defined with size dimensions. The dimensions are randomly generated, each number between 0 and 1. There are a random number of links, between 2 and 10. All of these constants can be easily changed, but were selected for both breadth and clarity.

**Joints:** Each 2 links have a joint between them with a planar degree of movement. Each joint is randomly assigned its degree of freedom.

![IMG_442A251F0BFC-1](https://user-images.githubusercontent.com/71985604/220263527-a7164093-e6b8-43c5-8e9e-426a4ff38929.jpeg)

**Sensors and Motors:** Sensors give signal as to whether the link is touching the ground or not. A random number of sensors (less than the random number of links) are placed at randomly selected positions and are indicated by green colored links. All links without sensors are colored cyan. All joints are motorized, and all sensors send inputs to all motors via synapses.

**Morphospace:** The generated bodies are capable of filling 3D space. A link can be placed on any unoccupied face of another link, such that the space it occupies at beginning of simulation does not intersect with any other link. Links must be placed face to face; not face to edge or face to corner.

![IMG_E72A29580D31-1](https://user-images.githubusercontent.com/71985604/220263679-d1a81747-8e17-44a1-85d9-6b1edc6e1091.jpeg)

**Mutation:** Robot bodies can evolve by resizing their blocks. They cannot add, remove, or change the position of their blocks, but even simple resizing can result in drastic mobility differences. (Robot links are not meant to intersect, but there is currently a bug in the code that resets the link's positions after a link is resized, so the collision detector meant to prevent link intersection had to be turned off temporarily.)

Update: I spent 2 days trying to fix the collider and after 2 days I had to give up again so that I could work on turning this in on time. Very sad.

![IMG_3798E712A9FC-1](https://user-images.githubusercontent.com/71985604/221772024-1aa4af7c-f545-4130-8d61-9c1395ae4535.jpeg)

**Evolution:** Robots were evolved using the parallel hill climber algorithm, where there is a set number of random seeds and each generation, one mutuation is made and a simulation is run. Each robot in a simulation is given a fitness score at the end, which in this case is the total distance traveled from the starting position. Parent and child fitness and whoever wins becomes the new parent for the next generation.

**Results:** These are the results of evolving 100 seeds for 500 generations. The following is a graph of how fitness increases (sorry the legend gets a little screwed up) and the youtube video link is to a highlight reel displaying diversity and mobility (and some fails). Honestly, there were some robots that didn't make significant improvement (as can be seen on the fitness graph) and I felt the need to go back and check my fitness function to see if it was actually being factored in sometimes. A stronger evolutionary algorithm is clearly needed, as I could have run only 250 gens and still come out with pretty similar results. Only a few seeds evolved late game. It also might have helped to have a working collider or have any sort of brain evolution (which I was going to add but ran out of time).
<img width="1246" alt="Screen Shot 2023-03-13 at 18 13 00" src="https://user-images.githubusercontent.com/71985604/224883533-d11e7b3c-9942-4e72-92c1-5a98b562ae41.png">

https://youtu.be/pDEtfqhaglg
