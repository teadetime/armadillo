import classification

cl = classification.classification()

# print(cl.fileWalker(cl.mlFileDir))
print(len(list((cl.fileWalker(cl.mlFileDir)))))

# cl.updateTrainingDataset()

cl.loadTrainingData()