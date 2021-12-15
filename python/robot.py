import math
import serialCmd
import time
class robot:
    """
    Commands and Repsonses(->)
    # MOVE
    <M, 24, 0, 0, 2, 0, 50> # Send a Move command IN STEPS
    -> <M, Y> Success
    -> <M, N> Failure

    # HOME
    <H, 0, 0, 0, 0, 0, 0> # ONLY THE H IS READ, starts a homing proceedure
    -> <H, Y> Success (Homing angles are known in Pyhton already obviously 0 steps maps to those angles)
    -> <M, N> Failure

    # INFO
    <I,> # INFO Get info of robot arm
    -> <I,j1,j2,j3,j4,vac,speed>

    # SETUP
    <S, microstep > # Other things that need to be changed? speed? Speed is same for all motors?
    -> <S, Y> # Microstepping set (should update speeds and accelerations)

    # CONTROL
    <C, Stop, Pause, DisableMotors > # Priority left to right set to 1 or zero, could be 0,1, or 2 also?
    -> <C, Y> # Control Done, should ech back what it did?

    """
    def __init__(self):
        self.serial = serialCmd.Serial_cmd()
        # if not self.serial.connected:
        #     print("Please Connect Arduino")
        #     quit()

        self.stepsPerRev = 200      # Number of steps in 1 rev of the steppers
        self.P1xyz = (0,0,110)       # Only the Z offset does anything
        self.L1 = 322               # mm length of first Arm Bearing to bearing
        self.L2 = 322               # 2nd Arm
        self.L3 = 17.5                # Length of rotating end effector
        self.maxExtension=550           # How far out can the arm go!
        #
        self.J1microSteps = 16
        self.J2microSteps = 16
        self.J3microSteps = 16
        self.J1gearing = 3
        self.J2gearing = 3
        self.J3gearing = 3

        # Radian values for the limit switches
        self.limitJ1 = math.pi/2 + math.radians(13.8)
        self.limitJ2 = math.radians(89)
        self.limitJ2min = math.radians(0)
        self.limitJ3 = math.pi/2+math.radians(92)
        self.limitJ3min = math.radians(85)

        self.lookingForBlock = True
        self.j4Offset = 0 # THis is used for non perpendicular picks!

        print(f"j1Limit: {self.limitJ1}\nj2Limit: {self.limitJ2}\nj3Limit: {self.limitJ3}")
        self.j1ZeroSteps = self.radToSteps(self.limitJ1, self.J1microSteps, self.J1gearing)
        self.j2ZeroSteps = self.radToSteps(self.limitJ2, self.J2microSteps, self.J2gearing)
        self.j3ZeroSteps = self.radToSteps(self.limitJ3, self.J3microSteps, self.J3gearing)

        # Offsets from Pivot point to servo Horn
        self.servoArmOffset = 32.5
        self.servoZOffset = 35

        self.suctionOffset = 0 # Also could be droop

        # This should just be steps each stepper is currently Commanded to? Not in Use
        self.theta1 = 0
        self.theta2 = 0
        self.theta3 = 0
        self.theta4 = 0
        self.lastObjective = ''

        # ETC
        self.commands = {}
        self.commands["info"] = 'I'
        self.commands["move"] = 'M'
        self.commands["home"] = 'H'
        self.commands["other"] = 'O'
        self.commands["control"] = 'C'
        self.commands["calibrate"] = 'B'

        # self.infoChar = 'I'
        # self.move = "M"
        # self.info = "I"
        # self.home = "H"
        # self.other = "O"
        # self.control = "C"
        # self.timeout = "T"
        self.handshakeChar = "A"
        self.successChar = "Y"
        self.failChar = "N"
        self.splitChar = ","
        self.startChar = '<'
        self.endChar = '>'


    def worldToJoint(self, coords, thetab):
        print("Block Rotation!:", thetab)
        '''
        Takes True world frame coordinates of the block! thetab is relative to block
        coords: tuple looks like -> (xb,yb,zb)
        '''

        # CHeck to see if the arm is going to a bad place!
        if coords[1] < -75 or 160 > math.sqrt(coords[0]**2 + coords[1]**2) or  math.sqrt(coords[0]**2 + coords[1]**2) > self.maxExtension:
            print("Cannot reach this position")
            return False

        armAngle = math.degrees(math.atan2(-coords[0], coords[1]))
        print(f"jOffset: {self.j4Offset}")
        if thetab-armAngle <=-90:
            print("Left Side")
            thetab += 180
        elif thetab-armAngle >= 90:
            thetab -= 180
            print("RightSide")

        if self.lookingForBlock:
            self.j4Offset = 0   # Reset the offset and see if we need a new offset for where we are going
             # Determine if this is a hard to pick spot

            print(f"Angle for looking at parallel picks:{abs(thetab-armAngle)}")
            # This means we should pick at a different angle!
            if abs(thetab-armAngle) > 80 and abs(thetab-armAngle < 100):
                #Left
                if coords[0]<0:
                    self.j4Offset = 45
                else: #Right
                    self.j4Offset = -45
        else: 
            # Keep using the offset that is already here
            pass
        # First lets assume we are choosing the position of the end effector ie 45*
        thetabRad = math.radians(thetab+self.j4Offset)


  

        # Calculate the xyz of arm
        #First work backwords from block to position the end of servo
        za = coords[2] + self.suctionOffset #Z_arm

        xa = coords[0] + self.L3*math.sin(thetabRad)
        ya = coords[1] - self.L3*math.cos(thetabRad)

        # Now calculate the position of the rotating joint of the arm
        za +=  self.servoZOffset

        # Time for Law of Cosines
        effectiveArmShadow = math.sqrt(xa**2 + ya**2)-self.servoArmOffset # imagine shinging a light directly above the arm
        zextra = (za-self.P1xyz[2]) # bottom of law of cosines trinagle

        b = math.sqrt(zextra**2 + effectiveArmShadow**2)

        gamma = math.atan2(zextra, effectiveArmShadow)

        # NOTE THETSE ARE BOTH Zero when they point straight out
        theta2 = math.acos((self.L1**2 + b**2 - self.L2**2)/(2*self.L2*b)) + gamma
        theta3 = math.acos((self.L1**2 - b**2 + self.L2**2)/(2*self.L1*self.L2)) + theta2
        theta1 = math.atan2(-xa, ya) # Flip the coordinates since we need to offset by pi/2 this allows us to handle negatives!

        #print(theta1, theta2, theta3)

        # if theta2 > self.limitJ2 or theta2 < self.limitJ2min:
        #     return None
        # if theta3 > self.limitJ3 or theta3 < self.limitJ3min:
        #     return None

       

        theta4 = -(theta1 - thetabRad)
        print("theta4:", math.degrees(theta4))
        return (theta1, theta2, theta3, theta4)

    def jointToWorld(self, stepPos):
        jPos = self.stepTupleToRadTuple(stepPos)

        theta1 = jPos[0]
        theta2 = jPos[1]
        theta3 = jPos[2]   # Angle of stepper arm to the angle of the triangle
        theta4 = jPos[3]

        thetaI = theta3 - (math.pi/2)

        #print(f"{theta1}, {theta2}, {theta3}, {theta4}, {thetaI}")
        middleJointZ = self.L1 * math.sin(theta2) - self.L2*math.cos(thetaI)
        middleJointArm = self.L1 * math.cos(theta2) + self.L2*math.sin(thetaI)

        # Now lets offset to servo, Please lets design this so the horn is in line
        servoZ = middleJointZ - self.servoZOffset
        servoArm = middleJointArm + self.servoArmOffset

        # Now lets decompose the arm
        servoHornX = servoArm * math.sin(-1*theta1)
        servoHornY = servoArm * math.cos(theta1)

        #Uncomment to look at the pivot point
        #print(f"{self.P1xyz[0] + servoHornX}, {self.P1xyz[1] + servoHornY}, {self.P1xyz[2] + servoZ}")

        # now do the rotation of the end effector
        eofRot = theta1 + theta4
        eofX = self.L3 * math.sin(eofRot)
        eofY = self.L3 * math.cos(eofRot)

        # Apply EOF Z adjustment
        finalX = self.P1xyz[0] + servoHornX - eofX
        finalY = self.P1xyz[1] + servoHornY + eofY
        finalZ = self.P1xyz[2] + servoZ - self.suctionOffset
        return (finalX,finalY,finalZ)

    def radTupleToStepTuple(self, jPos):
        if len(jPos) != 4:
            print("Error!")
            return
        else:
            step1 = self.radToSteps(jPos[0], self.J1microSteps, self.J1gearing)
            step2 = self.radToSteps(jPos[1], self.J2microSteps, self.J2gearing)
            step3 = self.radToSteps(jPos[2], self.J3microSteps, self.J3gearing)
            eofDegrees = math.degrees(jPos[3])
            return (int(step1), int(step2), int(step3), int(eofDegrees))

    def stepTupleToRadTuple(self, stepPos):
        if len(stepPos) != 4:
            print("Error!")
            return
        rad1 = self.stepsToRads(stepPos[0], self.J1microSteps, self.J1gearing)
        rad2 = self.stepsToRads(stepPos[1], self.J2microSteps, self.J2gearing)
        rad3 = self.stepsToRads(stepPos[2], self.J3microSteps, self.J3gearing)
        rad4 = math.radians(stepPos[3])
        return (rad1, rad2, rad3, rad4)

    def radToSteps(self, rad, microSteps, gearing, stepsRev = 200):
        '''
        Conversion from radians to a number of steps
        given ceratin steps per revolution and microstepping
        '''
        return rad/(2*math.pi) * stepsRev * microSteps * gearing

    def stepsToRads(self, steps, microSteps, gearing, stepsRev = 200):
        return steps/(stepsRev * microSteps * gearing) * (2*math.pi)

    def waitForResponse(self):
        while 1:
            if(self.serial.dev.in_waiting > 0):
                #print(f"bytes:{s.dev.in_waiting}")
                data = self.serial.read().strip()[1:-1] #gets data of form <I,435,45,4,3,3,2>
                #print(f"data: {data} type: {type(data)}")
                if not data: continue
                if (data[0] == self.commands["info"]):
                    # PARSE INFO INTO A LIST
                    data_tup = tuple('('+data+')')
                    #infoList.append(data_tup)
                    if(self.lastObjective == self.commands["info"]):
                        print(data[2:])
                        return data[2:]
                        break

                if(data[0] == self.lastObjective):
                    resultChar = data.split(self.splitChar)[-1]
                    return(resultChar == self.successChar)
                    break
            time.sleep(.02)
        return

    def waitForArduino(self):
        # Established serial connection
        while 1:
            var = self.serial.dev.read().decode()
            print(f"{var, type(var)}")
            if var == self.handshakeChar:
                self.serial.write(self.handshakeChar)
                print("Handshake Done!")
                self.serial.dev.reset_input_buffer() # may be uneccessary
                data = self.serial.read().strip()     # This line is essential and somehow reads the empty line that is output at the end of Setup
                break

    def createMessage(self, messageType, jPos, vac = 0, pump = 0):
        """
        Sends form startChar j1,j2,j3,j4,vac,speed endChar
        ie. <M,0,20,30,50,1.0,50>
        """
        # Make sure that jPos etc are trimmed to x decimals
        # Should we send angles or steps?
        self.lastObjective = messageType
        message = (self.startChar+messageType+self.splitChar+str(jPos[0])+self.splitChar+str(jPos[1])
                    +self.splitChar+str(jPos[2])+self.splitChar+str(jPos[3])
                    +self.splitChar+str(vac)+self.splitChar+str(pump)+self.endChar)
        return message


    def moveTo(self, x, y, z, theta, suction, pump):
        # Go to a position
        jPos = self.worldToJoint((x, y, z), theta)
        if not jPos:
            print("Arm is unable to get to reach this position!")
            return False
        stepPos = self.radTupleToStepTuple(jPos)
        nextPoint = self.createMessage(self.commands["move"], stepPos, vac = float(suction), pump = float(pump))
        self.serial.write(nextPoint)
        print(f"sent message: {nextPoint}")
        result = self.waitForResponse()
        print(result)
        return True

    def home(self):
        homeTuple = (self.j1ZeroSteps ,self.j2ZeroSteps, self.j3ZeroSteps, 0)
        homingMessage = self.createMessage(self.commands["home"],homeTuple,0,0)
        self.serial.write(homingMessage)
        print(f"Homing: {homingMessage}")
        result = self.waitForResponse()
        print(result)

    def calibrate(self):
        calibratingMessage = self.createMessage(self.commands["calibrate"],(0, 0, 0, 0),0,0)
        self.serial.write(calibratingMessage)
        print(f"Calibrating: {calibratingMessage}")
        result = self.waitForResponse()
        print(result)

    def controlVacPump(self, vac=1,pump=1):
        controlMessage = self.createMessage(self.commands["control"], (0, 0, 0, 0), int(vac), int(pump))
        self.serial.write(controlMessage)
        print(f"Calibrating: {controlMessage}")
        result = self.waitForResponse()
        print(result)

    def calcSmoothPick(self, coords, approachZ=5, stepSize=1, extract=False):
        """
        Calculates approach for the block using several points!
        """
        print("SMOOTH")
        x = coords[0]
        y = coords[1]
        z = coords[2]
        rotation = coords[3]

        tupleApproach = []

        steps = int(approachZ/stepSize)+1

        for i in range(steps):
            tupleApproach.append((x,y,z+i*stepSize,rotation)) 

        if not extract:
            return reversed(tupleApproach)
        return tupleApproach

    def calcSmoothPlace(self, coords, approachZ=15, approachTangent=20,steps = 15,direction=1, behind=False):
            rotation = coords[3]
            deltaX = approachTangent * math.sin(math.radians(rotation)) * direction
            deltaY = approachTangent * math.cos(math.radians(rotation)) * direction
            if behind:
                deltaX *= -1
                deltaY *= -1
            
            stepX = deltaX/steps
            stepY = deltaY/steps
            stepZ = approachZ/steps
            
            tupleApproach = []
            for i in range(steps):
                tupleApproach.append((coords[0]+i*stepX,coords[1]+i*stepY,coords[2]+i*stepZ,rotation)) 

            if not behind:
                return reversed(tupleApproach)
            return tupleApproach
