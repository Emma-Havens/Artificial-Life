from solution import SOLUTION
import constants as c
import copy
import os
import random
import numpy as np
import matplotlib.pyplot as plt
import pickle

class PARALLEL_HILL_CLIMBER:

    def __init__(self):
        self.parents = dict()
        self.nextAvailableId = 0

        self.bestOfEachGen = np.zeros((c.populationSize, c.numOfGens + 1))
        self.curGen = 0

        for i in range(c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvailableId)
            self.nextAvailableId += 1

    def Set_Child_Id(self, child):
        child.Set_Id(self.nextAvailableId)
        self.nextAvailableId += 1

    def Spawn(self):
        self.children = dict()
        for i in self.parents:
            self.children[i] = copy.deepcopy(self.parents[i])
            self.Set_Child_Id(self.children[i])

    def Mutate(self):
        for i in self.children:
            self.children[i].Mutate()

    def Print(self):
        for i in self.parents:
            print("Parent: " + str(self.parents[i].fitness) +
                  " child: " + str(self.children[i].fitness))
            #print("PARENT: ")
            #self.parents[i].printBodySize()
            #print("CHILD: ")
            #self.children[i].printBodySize()
        print("------")

    def Select(self):
        for i in self.parents:
            if (self.children[i].fitness > self.parents[i].fitness):
                self.parents[i] = self.children[i]

    def Save_Random(self):
        self.original = copy.deepcopy(self.parents[random.randint(0, c.populationSize - 1)])

    def LogGenFitness(self):
        for i in self.parents:
            self.bestOfEachGen[i][self.curGen] = self.parents[i].fitness
        self.curGen += 1

    def Show_Best(self):
        bestParent = 0
        for i in self.parents:
            if (self.parents[i].fitness > self.parents[bestParent].fitness):
                bestParent = i
        self.parents[bestParent].Start_Simulation("GUI")
        self.original.Start_Simulation("GUI")

    def Evaluate(self, solutions, connectionMode):
        for i in solutions:
            solutions[i].Start_Simulation(connectionMode)
        for i in solutions:
            solutions[i].Wait_For_Simulation_To_End()

    def Evolve_For_One_Gen(self):
        self.Spawn()
        self.Mutate()
        self.Evaluate(self.children, "DIRECT")
        #self.Print()
        self.Select()
        self.LogGenFitness()

    def Evolve(self):
        os.system("rm brain*.nndf")
        os.system("rm fitness*.txt")
        os.system("rm body*.urdf")
        self.Evaluate(self.parents, "DIRECT")
        #self.Save_Random()
        self.firstGen = copy.deepcopy((self.parents))
        self.LogGenFitness()
        for curGen in range(c.numOfGens):
            print("Evolving generation " + str(curGen))
            self.Evolve_For_One_Gen()

    def Display(self):
        for i in self.firstGen:
            self.firstGen[i].Start_Simulation("GUI")
            print(self.firstGen[i].fitness)
            self.children[i].Start_Simulation("GUI")
            print(self.children[i].fitness)
            self.firstGen[i].Wait_For_Simulation_To_End()
            self.children[i].Wait_For_Simulation_To_End()

    def PickleSim(self):
        pickle.dump(self.firstGen, open("firstGen.p", "wb"))
        pickle.dump(self.children, open("children.p", "wb"))
        pickle.dump(self.bestOfEachGen, open("bestOfEachGen.p", "wb"))

    def DisplayPickledSims(self):
        #firstGen = pickle.load(open("firstGen.p", "rb"))
        children = pickle.load(open("children.p", "rb"))
        for i in children:
            #firstGen[i].Start_Simulation("GUI")
            #print(firstGen[i].fitness)
            children[i].Start_Simulation("GUI")
            print(children[i].fitness)
            #firstGen[i].Wait_For_Simulation_To_End()
            children[i].Wait_For_Simulation_To_End()

    def DisplayPickledPlot(self):
        bestOfEachGen = pickle.load(open("bestOfEachGen.p", "rb"))
        for i in range(len(bestOfEachGen)):
            label = "seed" + str(i)
            plt.plot(bestOfEachGen[i], label=label)
        plt.xlabel("Generation")
        plt.ylabel("Fitness")
        plt.legend()
        plt.show()

    def ShowFitnessPlot(self):
        for i in range(len(self.bestOfEachGen)):
            label = "seed" + str(i)
            plt.plot(self.bestOfEachGen[i], label=label)
        plt.xlabel("Generation")
        plt.ylabel("Fitness")
        plt.legend()
        plt.show()
            