from solution import SOLUTION
import constants as c
import copy
import os

class PARALLEL_HILL_CLIMBER:

    def __init__(self):
        self.parents = dict()
        self.nextAvailableId = 0

        os.system("rm brain*.nndf")
        os.system("rm fitness*.txt")

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
        print("------")

    def Select(self):
        for i in self.parents:
            if (self.children[i].fitness < self.parents[i].fitness):
                self.parents[i] = self.children[i]

    def Show_Best(self):
        bestParent = 0
        for i in self.parents:
            if (self.parents[i].fitness < self.parents[bestParent].fitness):
                bestParent = i
        self.parents[bestParent].Start_Simulation("GUI")

    def Evaluate(self, solutions, connectionMode):
        for i in solutions:
            solutions[i].Start_Simulation(connectionMode)
        for i in solutions:
            solutions[i].Wait_For_Simulation_To_End()

    def Evolve_For_One_Gen(self):
        self.Spawn()
        self.Mutate()
        self.Evaluate(self.children, "DIRECT")
        self.Print()
        self.Select()

    def Evolve(self):
        self.Evaluate(self.parents, "DIRECT")
        for curGen in range(c.numOfGens):
            self.Evolve_For_One_Gen()
            