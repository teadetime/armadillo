import math
import robot
import vision
import time
import numpy as np

def towerPts(x0 = 0, y0 = 350, zOffset = 10, theta0 = 0, nLayers = 18):
            blockWidth = 20
            blockHeight = 15
            thetaOffset = 0 # -20
            theta = theta0 + thetaOffset

            for layer in range(nLayers):
                if layer % 2 == 0:
                    yield (x0 - blockWidth, y0, blockHeight * layer + zOffset, theta)
                    yield (x0,              y0, blockHeight * layer + zOffset, theta)
                    yield (x0 + blockWidth, y0, blockHeight * layer + zOffset, theta)

                if layer % 2 == 1:
                    yield (x0, y0 - blockWidth, blockHeight * layer + zOffset, theta + 90)
                    yield (x0, y0,              blockHeight * layer + zOffset, theta + 90)
                    yield (x0, y0 + blockWidth, blockHeight * layer + zOffset, theta + 90)


if __name__=='__main__':
    arm = robot.robot()
    vs = vision.vision()

    testingHomingandWorld = True
    testingCameras = False


    if testingHomingandWorld:
        #####################
        # Test Zero Position#
        #####################
        theta1ZeroSteps = arm.radToSteps(arm.limitJ1, arm.J1microSteps, arm.J1gearing)
        theta2ZeroSteps = arm.radToSteps(arm.limitJ2, arm.J2microSteps, arm.J2gearing)
        theta3ZeroSteps = arm.radToSteps(arm.limitJ3, arm.J3microSteps, arm.J3gearing)
        angle = 0
        block_angle = 0 + angle
        homeTuple = (theta1ZeroSteps,theta2ZeroSteps, theta3ZeroSteps, angle)

        # Test Main conversion
        # print(f"HomeJointTuple: {homeTuple}")
        # xyz = arm.jointToWorld(homeTuple)
        # print(f"World: {xyz}")
        # jPos = arm.worldToJoint(xyz, block_angle)
        # print(f"Reconvert: {arm.radTupleToStepTuple(jPos)}")

        # # Check for Arduinio
        # if not arm.serial.connected:
        #     print("Please Connect Arduino")
        #     quit()

        # arm.waitForArduino()

        ##############################
        ##Initiate Homing Proceedure##
        ##############################
        arm.home()

        #test = (-300, 300,10,0)
        # arm.moveTo(*test, suction = 0)
        arm.controlVacPump(1,1)
        time.sleep(2)

        top = (370, 370, 150,0)
        arm.moveTo(*top, suction = 1, pump=1)
        time.sleep(1)
        print("weirds")

        top = (-425, 80, 65,0)
        arm.moveTo(*top, suction = 1, pump=1)
        time.sleep(1)
        print("weirds")

        # testingPick = arm.calcSmoothPick(top, 6, 2)
        # for pos in testingPick:
        #     arm.moveTo(*pos, suction = 1, pump=1)

        # testingPick = arm.calcSmoothPick(test, 15, extract=True)
        # for pos in testingPick:
        #     arm.moveTo(*pos, suction = 1, pump=1)

        # testingPlace= arm.calcSmoothPlace(test)
        # print(list(testingPlace))
        # for pos in testingPlace:
        #     arm.moveTo(*pos, suction = 1, pump=1)

        # testingPlace= arm.calcSmoothPlace(test,direction=1, behind=False)
        # for pos in testingPlace:
        #     arm.moveTo(*pos, suction = 1, pump=1)

        # arm.controlVacPump(0,0)

        # testingPlace= arm.calcSmoothPlace(test,approachTangent=0,direction=-1, behind=True)
        # for pos in testingPlace:
        #     arm.moveTo(*pos, suction = 0, pump=0)
        ##testingPick = arm.calcSmoothPick((0,300,15,0), extract=True)

        arm.controlVacPump(1,1)


        quit()
        #Try Claibrating
        # calibTuple = (-175, 250, 15)
        # radsTuple = arm.worldToJoint(calibTuple, 0)
        # stepsTuple = arm.radTupleToStepTuple(radsTuple)
        # calibMessage = arm.createMessage(arm.commands["calibrate"],stepsTuple,0,0)
        # arm.serial.write(calibMessage)
        # print(f"Homing: {calibMessage}")
        # result = arm.waitForResponse()




        if testingCameras:
            if not vs.testCamera():
                print("Camera Not working")

            grabbingFrame = True
            while grabbingFrame:
                grabImageSuccess = vs.grabImage(fromPath=False)
                if not grabImageSuccess:
                    print("Please reposition Camera and check masking!")
                    vs.tuneWindow()
                    x = input('Retry (R) or Quit (Q): ')
                    if x == 'R':
                        pass
                    else:
                        quit()
                else:
                    grabbingFrame = False
            (coords, rotation) = vs.getBlockWorld()
            print(coords)
            print(rotation)


            #Go to a position
            testPoint = (coords[0],coords[1], 14, rotation)
            arm.moveTo(*testPoint, suction = 0)
            testPoint = (coords[0],coords[1], 5, rotation)
            arm.moveTo(*testPoint, suction = 1)

            testPoint = (coords[0],coords[1], 40, rotation)
            arm.moveTo(*testPoint, suction = 1)

            t = towerPts()
            testPoint = next(t)

            testPoint = (300,400, 25, 0)
            arm.moveTo(*testPoint, suction = 1)
            testPoint = (300,400, 15, 0)
            arm.moveTo(*testPoint, suction = 1)
            arm.moveTo(*testPoint, suction = 0)

            testPoint = (300,400, 40, 0)
            arm.moveTo(*testPoint, suction = 0)

            # testPoint[2] = testPoint[2] + 5
            # arm.moveTo(*testPoint, suction = 0)

        # testPoint = (-300, 400, 15, 0)
        # arm.moveTo(*testPoint, suction = 0)
        # time.sleep(5)
        # testPoint = (-300, 400, 45, 0)
        # arm.moveTo(*testPoint, suction = 0)

        # testPoint = (0, 400, 15, 0)
        # arm.moveTo(*testPoint, suction = 0)
        # time.sleep(5)
        # testPoint = (0, 400, 45, 0)
        # arm.moveTo(*testPoint, suction = 0)

        # testPoint = (300, 400, 15, 0)
        # arm.moveTo(*testPoint, suction = 0)
        # time.sleep(5)
        # testPoint = (300, 400, 45, 0)
        # arm.moveTo(*testPoint, suction = 0)
        # arm.home()
        # quit()

        # ORDER OF OPERATIONS:
        # 1. Home
        # 2. For each block, stackPosition in zip(blockList, stackPosition):
        #     a. go to midpoint
        #         suction off
        #     b. go to block
        #         suction on
        #     c. go to midpoint
        #         suction on
        #     d. go to stackPosition (maybe with some special finangling to avoid disturbing prior blocks)
        #         suction on
        #         suction off
        # 3. Home

        #######Points (X ,  Y ,  Z , R)
        ##testPoint = (0, 400, 10, 0)

        # testPoint = (0, 400, 10, 0)
        # arm.moveTo(*testPoint, suction = 0)
        # time.sleep(5)
        # exit()
        def testGridPts(xPts = 12,yPts=15,z=-5,r=0,yMin = 7, spacing = 25.4, pause = 0.5):
            for dx in range(-xPts,xPts):
                newX = dx*spacing
                for dy in range(yMin,yPts):
                    newY = dy*spacing
                    #TODO: CHECK TO SEE IF BAD POSITION!!
                    #continue
                    print(dx, dy)
                    testPoint = (newX, newY, z, r)

                    if not arm.moveTo(*testPoint, suction = 0):
                        continue
                    time.sleep(pause)

        def testGrid(x=305,y=400,z=15,r=0,yMin = 200, spacing = 25.4, pause = .25):
            for dx in range(-x,x, spacing):
                for dy in range(yMin,y, spacing):
                    #TODO: CHECK TO SEE IF BAD POSITION!!
                    #continue
                    testPoint = (dx, dy, z, r)
                    if not arm.moveTo(*testPoint, suction = 0):
                        continue
                    time.sleep(pause)


        #arm.moveTo(0, 350, 30, 0, 0)
        # testGridPts()
        # exit()



        # for towerPt in towerPts():
        #     arm.moveTo(*towerPt, 0)
        #     arm.moveTo(*towerPt, 0) # we need to test the function that just turns suction off

        for i in range(-45, 45, 15):
            arm.moveTo(0, 350, 30, i, 0)
        arm.moveTo(0, 400, 30, 0, 0)

        #arm.home()
        exit()



        # ##############################
        # ## Just Calibrate , For Now ##
        # ##############################
        # homeTuple = (arm.j1ZeroSteps ,arm.j2ZeroSteps, arm.j3ZeroSteps, 0)
        # homingMessage = arm.createMessage(arm.commands["calibrate"],(0, 0, 0, 0),0,0)
        # arm.serial.write(homingMessage)
        # print(f"Homing: {homingMessage}")
        # result = arm.waitForResponse()
        # print(result)
        # exit()
        # raise ValueError # stop the program

        # jPos = arm.worldToJoint((0,350, 5), 0)
        # stepPos = arm.radTupleToStepTuple(jPos)
        # nextPoint = arm.createMessage(arm.commands["move"],stepPos,0.0,40.0)
        # arm.serial.write(nextPoint)
        # print(f"sent message: {nextPoint}")
        # result = arm.waitForResponse()
        # print(result)
        # time.sleep(.5)

        # Go to a position
        jPos = arm.worldToJoint((coords[0],coords[1], 18), rotation)
        stepPos = arm.radTupleToStepTuple(jPos)
        nextPoint = arm.createMessage(arm.commands["move"],stepPos,0.0,40.0)
        arm.serial.write(nextPoint)
        print(f"sent message: {nextPoint}")
        result = arm.waitForResponse()
        print(result)
        time.sleep(.5)

        jPos = arm.worldToJoint((coords[0],coords[1], 5), rotation)
        stepPos = arm.radTupleToStepTuple(jPos)
        nextPoint = arm.createMessage(arm.commands["move"],stepPos,1.0,40.0)
        arm.serial.write(nextPoint)
        print(f"sent message: {nextPoint}")
        result = arm.waitForResponse()
        print(result)
        time.sleep(.5)

        jPos = arm.worldToJoint((coords[0],coords[1], 40), 0)
        stepPos = arm.radTupleToStepTuple(jPos)
        nextPoint = arm.createMessage(arm.commands["move"],stepPos,1.0,40.0)
        arm.serial.write(nextPoint)
        print(f"sent message: {nextPoint}")
        result = arm.waitForResponse()
        print(result)
        time.sleep(.5)

        jPos = arm.worldToJoint((-300,300, 20), 0)
        stepPos = arm.radTupleToStepTuple(jPos)
        nextPoint = arm.createMessage(arm.commands["move"],stepPos,1.0,40.0)
        arm.serial.write(nextPoint)
        print(f"sent message: {nextPoint}")
        result = arm.waitForResponse()
        print(result)
        time.sleep(.5)


        jPos = arm.worldToJoint((-300,300, 10), 0)
        stepPos = arm.radTupleToStepTuple(jPos)
        nextPoint = arm.createMessage(arm.commands["move"],stepPos,1.0,40.0)
        arm.serial.write(nextPoint)
        print(f"sent message: {nextPoint}")
        result = arm.waitForResponse()
        print(result)
        time.sleep(.5)

        jPos = arm.worldToJoint((-300,300, 40), 0)
        stepPos = arm.radTupleToStepTuple(jPos)
        nextPoint = arm.createMessage(arm.commands["move"],stepPos,0,40.0)
        arm.serial.write(nextPoint)
        print(f"sent message: {nextPoint}")
        result = arm.waitForResponse()
        print(result)
        time.sleep(.5)
        #  # # Go to a position
        # jPos = arm.worldToJoint((-0,400, 40), 0)
        # stepPos = arm.radTupleToStepTuple(jPos)
        # nextPoint = arm.createMessage(arm.commands["move"],stepPos,0,40.0)
        # arm.serial.write(nextPoint)
        # print(f"sent message: {nextPoint}")
        # result = arm.waitForResponse()
        # print(result)
        # time.sleep(.5)

        # # Go to a position
        # jPos = arm.worldToJoint((0,300, 30), 0)
        # stepPos = arm.radTupleToStepTuple(jPos)
        # nextPoint = arm.createMessage(arm.commands["move"],stepPos,1.0,40.0)
        # arm.serial.write(nextPoint)
        # print(f"sent message: {nextPoint}")
        # result = arm.waitForResponse()
        # print(result)
        # time.sleep(.5)

        # homingMessage = arm.createMessage(arm.commands["home"],homeTuple,0,0)
        # arm.serial.write(homingMessage)
        # print(f"Homing: {homingMessage}")
        # result = arm.waitForResponse()
        # print(result)

    #for jPos in jPosList:
        # Calculate position
        # layer = jengaBlock // 3 + 1
        # rotation = (layer % 2) * 90
        # position = (jengaBlock) % 3
        # jPos = (jengaBlock*10,0,0,0)

        # print(f"Working on Block:{jengaBlock+1} Layer:{layer}, rotation:{rotation}, position {position} ")
        #nextPoint = createMessage(move,jPos,1.0,40.0)
        #print(lastObjective)
        # s.write(nextPoint)
        # print(f"sent message: {nextPoint}")
        # result = waitForResponse()
        #time.sleep(1)
