# read all files and store in a big matrix
# define subclasses
#

import numpy as np
import os
import fnmatch

class classification:

    def __init__(self):
        self.trainingDataFile = ".\\machineLearning\\completeTrainingDataMatrix.npy"
        self.truthFile = ".\\machineLearning\\completeTruthFile.npy"
        self.mlFileDir = "C:\\Users\\ieykamp\\MATLAB Drive\\PIE\\mlData"
        self.mlTruthDir = "C:\\Users\\ieykamp\\MATLAB Drive\\PIE\\mlTruth"

    def fileWalker(self, root):
        dir, rel, fileNames = next(os.walk(root))
        for file in fnmatch.filter(fileNames, "*.csv"):
            completeFileName = os.path.join(dir, file)
            print(completeFileName)
            yield completeFileName

    def updateTrainingDataset(self):
        self.dataMatrix = np.array([])
        self.truthList = np.array([])

        # for i, file in enumerate(self.fileWalker(self.mlFileDir)):
        #     newCols = np.genfromtxt(file, delimiter = ',').astype(int)
        #     if i == 0:
        #         self.dataMatrix = newCols
        #     else:
        #         print(np.shape(self.dataMatrix))
        #         self.dataMatrix = np.vstack((self.dataMatrix, newCols))

        for i, file in enumerate(self.fileWalker(self.mlTruthDir)):
            newVals = np.genfromtxt(file, delimiter=',').astype(int)
            if i == 0:
                self.truthLlist = newVals
            else:
                print(newVals)
                self.truthList = np.append(self.truthList, newVals)

        # np.save(self.trainingDataFile, self.dataMatrix)
        np.save(self.truthFile, self.truthList)

    def loadTrainingData(self):
        self.dataMatrix = np.load(self.trainingDataFile)
        self.truthList = np.load(self.truthFile)

    def computeEigenvectors():
        pass