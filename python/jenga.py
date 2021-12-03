import math
import robot
import vision
import time

if __name__=='__main__':
    arm = robot.robot()
    vs = vision.vision()

    testingHomingandWorld = True
    testingCameras = False

    """
    COde to Detect basis and camera tags
    """
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
        vs.getBlockWorld()

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
        print(f"HomeJointTuple: {homeTuple}")
        xyz = arm.jointToWorld(homeTuple)
        print(f"World: {xyz}")
        jPos = arm.worldToJoint(xyz, block_angle)
        print(f"Reconvert: {arm.radTupleToStepTuple(jPos)}")

        # Check for Arduinio
        if not arm.serial.connected:
            print("Please Connect Arduino")
            quit()

        arm.waitForArduino()





        ##############################
        ##Initiate Homing Proceedure##
        ##############################
        homeTuple = (arm.j1ZeroSteps ,arm.j2ZeroSteps, arm.j3ZeroSteps, 0)
        homingMessage = arm.createMessage(arm.commands["home"],homeTuple,0,0)
        arm.serial.write(homingMessage)
        print(f"Homing: {homingMessage}")
        result = arm.waitForResponse()
        print(result)


        # Go to a position
        jPos = arm.worldToJoint((-350,350, 0), 0)
        stepPos = arm.radTupleToStepTuple(jPos)
        nextPoint = arm.createMessage(arm.commands["move"],stepPos,1.0,40.0)
        arm.serial.write(nextPoint)
        print(f"sent message: {nextPoint}")
        result = arm.waitForResponse()
        print(result)
        time.sleep(.5)

        # Go to a position
        jPos = arm.worldToJoint((0,425, 50), 0)
        stepPos = arm.radTupleToStepTuple(jPos)
        nextPoint = arm.createMessage(arm.commands["move"],stepPos,1.0,40.0)
        arm.serial.write(nextPoint)
        print(f"sent message: {nextPoint}")
        result = arm.waitForResponse()
        print(result)
        time.sleep(.5)

        # Go to a position
        jPos = arm.worldToJoint((0,425, 0), 0)
        stepPos = arm.radTupleToStepTuple(jPos)
        nextPoint = arm.createMessage(arm.commands["move"],stepPos,1.0,40.0)
        arm.serial.write(nextPoint)
        print(f"sent message: {nextPoint}")
        result = arm.waitForResponse()
        print(result)
        time.sleep(.5)

        homingMessage = arm.createMessage(arm.commands["home"],homeTuple,0,0)
        arm.serial.write(homingMessage)
        print(f"Homing: {homingMessage}")
        result = arm.waitForResponse()
        print(result)
    # # Go to a position
    # jPos = arm.worldToJoint((0,280, 300), 0)
    # stepPos = arm.radTupleToStepTuple(jPos)
    # nextPoint = arm.createMessage(arm.commands["move"],stepPos,1.0,40.0)
    # arm.serial.write(nextPoint)
    # print(f"sent message: {nextPoint}")
    # result = arm.waitForResponse()
    # print(result)

    # # Go to a position
    # jPos = arm.worldToJoint((100,280, 300), 0)
    # stepPos = arm.radTupleToStepTuple(jPos)
    # nextPoint = arm.createMessage(arm.commands["move"],stepPos,1.0,40.0)
    # arm.serial.write(nextPoint)
    # print(f"sent message: {nextPoint}")
    # result = arm.waitForResponse()
    # print(result)




    # jPos = worldToJoint((100,220, 120), 0)
    # stepPos = radTupleToSteptuple(jPos)
    # nextPoint = createMessage(move,stepPos,1.0,40.0)
    # s.write(nextPoint)
    # print(f"sent message: {nextPoint}")
    # result = waitForResponse()


    # jPosList = [
    #     (-0,0,0,0),
    #     (-0,0,10,0),
    #     (-0,0,0,0),
        # Nicer demo code
        # (-0,0,0,0),
        # (400, -100, 200,0),
        # (0,0,0,0),
        # (-400, -250, 0,0),
        # (0,0,0,0),
        # (0,0,300,0),
        # (0,0,0,0),
        #2d DMEO CODE
        # (-0,0,0,0),
        # (100, 200,0,0),
        # (0,0,0,0),
        # (300, 250,0,0),
        # (0,0,0,0),
                # 32 microstepping
                # (-0,0,0,0),
                # (600, 1000,0,0),
                # (0,0,0,0),
                # (800, 600,0,0),
                # (0,0,0,0),
                # (1000, 600,0,0),
                # (0,0,0,0),

                # ]*2
    #for jengaBlock in range(54):

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
