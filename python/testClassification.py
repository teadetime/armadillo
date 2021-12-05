import numpy as np
import scipy.io

import classification

cl = classification.classification()

# print(cl.fileWalker(cl.mlFileDir))
print(len(list((cl.fileWalker(cl.mlFileDir)))))

cl.updateTrainingDataset()

cl.loadTrainingData()

print(np.shape(cl.dataMatrix))
print(np.shape(cl.truthList))

mat = scipy.io.loadmat("C:\\Users\\ieykamp\\MATLAB Drive\\PIE\\truthList.mat")["truthList"]
print(mat)
print(type(mat))
print(np.shape(mat))
print(mat - cl.truthList)