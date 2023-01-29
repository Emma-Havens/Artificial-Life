import os
from parallelhillclimber import PARALLEL_HILL_CLIMBER

phc = PARALLEL_HILL_CLIMBER()
phc.Evolve()
phc.Show_Best()



#for i in range(5):
#    os.system("python generate.py")
#    os.system("python simulate.py")