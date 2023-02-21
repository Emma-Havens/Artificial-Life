import numpy as np

class LINK:

    def __init__(self, linkName, posArr, sizeArr, prevLink=None, faceDir=None, startingz=None):
        self.linkName = linkName
        #this is wrt previous joint
        self.posArr = posArr
        self.sizeArr = sizeArr
        self.prevLink = prevLink
        if (prevLink == None):
            self.absPosArr = np.array([0, 0, startingz])
        else:
            prevAbsPosArr = prevLink.absPosArr
            prevSizeArr = prevLink.sizeArr
            if (faceDir == 1):
                self.absPosArr = np.array([prevAbsPosArr[0],
                                           prevAbsPosArr[1],
                                           prevAbsPosArr[2] + (prevSizeArr[2] / 2) + (self.sizeArr[2] / 2)])
            elif (faceDir == 2):
                self.absPosArr = np.array([prevAbsPosArr[0],
                                           prevAbsPosArr[1] - (prevSizeArr[1] / 2) - (self.sizeArr[1] / 2),
                                           prevAbsPosArr[2]])
            elif (faceDir == 3):
                self.absPosArr = np.array([prevAbsPosArr[0],
                                           prevAbsPosArr[1],
                                           prevAbsPosArr[2] - (prevSizeArr[2] / 2) - (self.sizeArr[2] / 2)])
            elif (faceDir == 4):
                self.absPosArr = np.array([prevAbsPosArr[0],
                                           prevAbsPosArr[1] + (prevSizeArr[1] / 2) + (self.sizeArr[1] / 2),
                                           prevAbsPosArr[2]])
            elif (faceDir == 5):
                self.absPosArr = np.array([prevAbsPosArr[0] + (prevSizeArr[0] / 2) + (self.sizeArr[0] / 2),
                                           prevAbsPosArr[1],
                                           prevAbsPosArr[2]])
        #print(self.absPosArr)
        self.xmin = self.absPosArr[0] + self.sizeArr[0] / -2
        self.xmax = self.absPosArr[0] + self.sizeArr[0] / 2
        self.ymin = self.absPosArr[1] + self.sizeArr[1] / -2
        self.ymax = self.absPosArr[1] + self.sizeArr[1] / 2
        self.zmin = self.absPosArr[2] + self.sizeArr[2] / -2
        self.zmax = self.absPosArr[2] + self.sizeArr[2] / 2

        #print("(" + str(self.xmin) + ", " + str(self.xmax) + ")")
        #print("(" + str(self.ymin) + ", " + str(self.ymax) + ")")
        #print("(" + str(self.zmin) + ", " + str(self.zmax) + ")")


    def NoCollision(self, otherLink):
        return (self.xmin >= otherLink.xmax or self.xmax <= otherLink.xmin) or \
               (self.ymin >= otherLink.ymax or self.ymax <= otherLink.ymin) or \
               (self.zmin >= otherLink.zmax or self.zmax <= otherLink.zmin)
