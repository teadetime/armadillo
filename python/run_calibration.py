# run calibration
import os
import cv2
import Ian_vision

print(os.walk("C:\\Users\\ieykamp\\MATLAB Drive\\PIE\\mlImages"))
mlFileDir = "C:\\Users\\ieykamp\\MATLAB Drive\\PIE\\mlData"
mlTruthDir = "C:\\Users\\ieykamp\\MATLAB Drive\\PIE\\mlTruth"

def processedFiles(imFilenames):

    existingDir, rel, existingFilenames = next(os.walk("C:\\Users\\ieykamp\\MATLAB Drive\\PIE\\mlData"))

    for imFile in imFilenames:
        csvFile = imFile[:-4] + ".csv"
        if csvFile in existingFilenames:
            continue
        if imFile == ".MATLABDriveTag":
            continue
        yield imFile, csvFile

for imDir, rel, imFilenames in os.walk("C:\\Users\\ieykamp\\MATLAB Drive\\PIE\\mlImages"):
    for imFile, csvFile in processedFiles(imFilenames):

        print(os.path.join(imDir, imFile))
        vs = Ian_vision.vision()
        vs.grabImage(path = os.path.join(imDir, imFile), pickSwatches = True)
        vs.getBlockPixel(mlFile = os.path.join(mlFileDir, csvFile), truthFile = os.path.join(mlTruthDir, csvFile))
        print("finished with one file")
        cv2.destroyAllWindows()

