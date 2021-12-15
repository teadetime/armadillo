import math
import robot
# import vision
import vision as vision
import time
import numpy as np

def towerPts(x0 = 0, y0 = 350, zOffset = 0, theta0 = -90, nLayers = 18):
            blockWidth = 23
            blockHeight = 15
            thetaOffset = 0 # -20
            theta = theta0 + thetaOffset

            for layer in range(nLayers):
                if layer % 2 == 0:
                    yield (x0 + blockWidth, y0, blockHeight * layer + zOffset, theta)
                    yield (x0,              y0, blockHeight * layer + zOffset, theta)
                    yield (x0 - blockWidth, y0, blockHeight * layer + zOffset, theta)

                if layer % 2 == 1:
                    yield (x0, y0 - blockWidth, blockHeight * layer + zOffset, theta + 90)
                    yield (x0, y0,              blockHeight * layer + zOffset, theta + 90)
                    yield (x0, y0 + blockWidth, blockHeight * layer + zOffset, theta + 90)


if __name__=='__main__':
    arm = robot.robot()
    vs = vision.vision()

    testingHomingandWorld = True
    testingCameras = True


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

        # Check for Arduinio
        if not arm.serial.connected:
            print("Please Connect Arduino")
            quit()

        arm.waitForArduino()

        ##############################
        ##Initiate Homing Proceedure##
        ##############################
        arm.home()

        # test = (0, 400,10,0)
        # arm.moveTo(*test, suction = 0, pump=0)

        time.sleep(5)

        arm.controlVacPump(1,1)
        t = towerPts(x0=200,y0=350, zOffset=6)
        for i in range(54):
            placePoint = next(t)
            print(placePoint)

            arm.lookingForBlock = True
            # Perch
            perch = (-100, 300, placePoint[2]+70, 0)
            arm.moveTo(*perch, suction = 1, pump=1)

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
            (coords, rotation) = next(vs.getBlockWorld())
            print(coords)
            print(rotation)

            block = (coords[0],coords[1], 3,rotation)
            # arm.moveTo(*top, suction = 1, pump=1)
            # time.sleep(1)
            # print("weirds")
            grabPlace = arm.calcSmoothPlace(block, approachZ=10, approachTangent=0,steps=2)
            for pos in grabPlace:
                arm.moveTo(*pos, suction = 1, pump=1)
                arm.lookingForBlock = False

            # Extract
            grabPlace = arm.calcSmoothPlace(block, approachZ=30, approachTangent=0,steps=2, behind=-1)
            print(list(grabPlace))
            for pos in grabPlace:
                arm.moveTo(*pos, suction = 1, pump=1)


            # Perch
            placeAngle = math.atan2(100,200)
            print("Angles")
            print(placeAngle, placePoint[3])
            # Perch
            perch = (-100, 250, placePoint[2]+60,placePoint[3]+placeAngle)

            arm.moveTo(*perch, suction = 1, pump=1)

            # PLace
            testingPlace= arm.calcSmoothPlace(placePoint,direction=1,steps=3, behind=False)
            for pos in testingPlace:
                arm.moveTo(*pos, suction = 1, pump=1)

            arm.controlVacPump(0,0)
            testingPlace= arm.calcSmoothPlace(placePoint,approachTangent=0,steps=2, behind=True)
            for pos in testingPlace:
                arm.moveTo(*pos, suction = 0, pump=0)

            arm.lookingForBlock = True
             




        arm.home()
        arm.controlVacPump(0,0)


        quit()
        #Try Claibrating
        # calibTuple = (-175, 250, 15)
        # radsTuple = arm.worldToJoint(calibTuple, 0)
        # stepsTuple = arm.radTupleToStepTuple(radsTuple)
        # calibMessage = arm.createMessage(arm.commands["calibrate"],stepsTuple,0,0)
        # arm.serial.write(calibMessage)
        # print(f"Homing: {calibMessage}")
        # result = arm.waitForResponse()



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

    # Camera Testing
    else:
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
