import serial
import serial.tools.list_ports as list_ports
import string, array
from datetime import datetime
import time
import math

class Serial_cmd:
    Arduino_IDs = ((0x2341, 0x0043), (0x2341, 0x0001), 
                   (0x2A03, 0x0043), (0x2341, 0x0243), 
                   (0x0403, 0x6001), (0x1A86, 0x7523))

    def __init__(self, port = ''):
        if port == '':
            self.dev = None
            self.connected = False
            devices = list_ports.comports()
            for device in devices:
                if (device.vid, device.pid) in Serial_cmd.Arduino_IDs:
                    try:
                        self.dev = serial.Serial(device.device, 115200)
                        self.connected = True
                        print('Connected to {!s}...'.format(device.device))
                    except:
                        pass
                if self.connected:
                    break
        else:
            try:
                self.dev = serial.Serial(port, 115200)
                self.connected = True
            except:
                self.dev = None
                self.connected = False

    def write(self, command):
        if self.connected:
            self.dev.write('{!s}\r'.format(command).encode())

    def read(self):
        if self.connected:
            return self.dev.readline().decode()

    # def get_yellow(self):
    #     if self.connected:
    #         self.write('YELLOW?')
    #         return int(self.read(), 16)

    # def set_yellow(self, val):
    #     if self.connected:
    #         self.write('YELLOW!{:X}'.format(int(val)))


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

class robot:
    def __init__(self):
        self.serial = Serial_cmd()
        # if not self.serial.connected:
        #     print("Please Connect Arduino")
        #     quit()

        self.stepsPerRev = 200      # Number of steps in 1 rev of the steppers
        self.P1xyz = (0,0,85)       # Only the Z offset does anything
        self.L1 = 320               # mm length of first Arm Bearing to bearing
        self.L2 = 320               # 2nd Arm
        self.L3 = 0                 # Length of rotating end effector

        # Radian values for the limit switches
        self.limitJ1 = 0
        self.limitJ2 = math.pi/4
        self.limitJ3 = 3*math.pi/4

        # Offsets from Pivot point to servo Horn
        self.servoArmOffset = 32
        self.servoZOffset = 15

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
        self.J1microSteps = 16
        self.J2microSteps = 16
        self.J3microSteps = 16        
        self.J1gearing = 3
        self.J2gearing = 3
        self.J3gearing = 3

    def worldToJoint(self, coords , thetab):
        '''
        Takes True world frame coordinates of the block! thetab is relative to block
        coords: tuple looks like -> (xb,yb,zb) 
        '''
        # First lets assume we are choosing the position of the end effector ie 45*
        thetabRad = thetab

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
        theta1 = math.atan2(ya, xa) - math.pi/2
        theta4 = thetabRad-theta1

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
            step4 = jPos[3]
            return (int(step1), int(step2), int(step3), int(step4))

    def stepTupleToRadTuple(self, stepPos):
        if len(stepPos) != 4:
            print("Error!")
            return
        rad1 = self.stepsToRads(stepPos[0], self.J1microSteps, self.J1gearing)
        rad2 = self.stepsToRads(stepPos[1], self.J2microSteps, self.J2gearing)
        rad3 = self.stepsToRads(stepPos[2], self.J3microSteps, self.J3gearing)
        rad4 = stepPos[3]
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
            time.sleep(.05)
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

    def createMessage(self, messageType, jPos, vac = 0, speed = 0):
        """
        Sends form startChar j1,j2,j3,j4,vac,speed endChar
        ie. <M,0,20,30,50,1.0,50>
        """
        # Make sure that jPos etc are trimmed to x decimals
        # Should we send angles or steps?
        self.lastObjective = messageType
        message = (self.startChar+messageType+self.splitChar+str(jPos[0])+self.splitChar+str(jPos[1])
                    +self.splitChar+str(jPos[2])+self.splitChar+str(jPos[3])
                    +self.splitChar+str(vac)+self.splitChar+str(speed)+self.endChar)
        return message


if __name__=='__main__':
    arm = robot()

    #####################
    # Test Zero Position#
    #####################
    theta1ZeroSteps = arm.radToSteps(arm.limitJ1, arm.J1microSteps, arm.J1gearing)
    theta2ZeroSteps = arm.radToSteps(arm.limitJ2, arm.J2microSteps, arm.J2gearing)
    theta3ZeroSteps = arm.radToSteps(arm.limitJ3, arm.J3microSteps, arm.J3gearing)
    angle = 0
    block_angle = 0 + angle
    homeTuple = (theta1ZeroSteps,theta2ZeroSteps, theta3ZeroSteps, angle)

    print(f"HomeJointTuple: {homeTuple}")
    xyz = arm.jointToWorld(homeTuple)
    print(f"World: {xyz}")
    jPos = arm.worldToJoint(xyz, block_angle)
    print(f"Reconvert: {arm.radTupleToStepTuple(jPos)}")
    

    if not arm.serial.connected:
        print("Please Connect Arduino")
        quit()

    arm.waitForArduino()
    




    # Initiate Homing Proceedure
    # nextPoint = createMessage(home)
    # s.write(nextPoint)
    # result = waitForResponse()

    jPosList = [
        (-0,0,0,0),
        (-0,0,10,0),
        (-0,0,0,0),
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
                
                ]*2
    #for jengaBlock in range(54):
    theta1ZeroSteps = arm.radToSteps(0, arm.J1microSteps, arm.J1gearing)
    theta2ZeroSteps = arm.radToSteps(math.pi/2,arm.J2microSteps, arm.J1gearing)
    theta3ZeroSteps = arm.radToSteps(3*math.pi/4,arm.J3microSteps, arm.J1gearing)
    homeTuple = (theta1ZeroSteps,theta2ZeroSteps, theta3ZeroSteps, 0)
    homingMessage = arm.createMessage(arm.commands["home"],homeTuple,0,0)
    arm.serial.write(homingMessage)
    print(f"Homing: {homingMessage}")
    result = arm.waitForResponse()
    print(result)


    #Move j1
    # Go to a position
    nextPoint = arm.createMessage(arm.commands["move"],(0,1500, 3600,0),1.0,40.0)
    arm.serial.write(nextPoint)
    print(f"sent message: {nextPoint}")
    result = arm.waitForResponse()
    print(result)
    #Move j2
    # Go to a position
    nextPoint = arm.createMessage(arm.commands["move"],(0,1500, 4000,0),1.0,40.0)
    arm.serial.write(nextPoint)
    print(f"sent message: {nextPoint}")
    result = arm.waitForResponse()
    print(result)



    # # Go to a position
    # jPos = arm.worldToJoint((0,352, 350), 0)
    # stepPos = arm.radTupleToStepTuple(jPos)
    # nextPoint = arm.createMessage(arm.commands["move"],stepPos,1.0,40.0)
    # arm.serial.write(nextPoint)
    # print(f"sent message: {nextPoint}")
    # result = arm.waitForResponse()
    # print(result)

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


    # Go home
    nextPoint = arm.createMessage(arm.commands["move"],homeTuple,1.0,40.0)
    arm.serial.write(nextPoint)
    print(f"sent message: {nextPoint}")
    result = arm.waitForResponse()
    print(result)

    # jPos = worldToJoint((100,220, 120), 0)
    # stepPos = radTupleToSteptuple(jPos)
    # nextPoint = createMessage(move,stepPos,1.0,40.0)
    # s.write(nextPoint)
    # print(f"sent message: {nextPoint}")
    # result = waitForResponse()

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
