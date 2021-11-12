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

####################
##Global Constants##
####################
move = "M"
info = "I"
home = "H"
other = "O"
control = "C"
timeout = "T"
successChar = "Y"
failChar = "N"
splitChar = ","

infoList = []
lastObjective = ''
microSteps = 32

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
def createMessage(messageType, jPos=(0,0,0,0), vac = 0, speed = 0, startChar = '<', endChar = '>', sep = ','):
    """
    Sends form startChar j1,j2,j3,j4,vac,speed endChar
    ie. <M,0,20,30,50,1.0,50>
    """
    # Make sure that jPos etc are trimmed to x decimals
    # Should we send angles or steps?
    global lastObjective
    lastObjective = messageType
    message = (startChar+messageType+sep+str(jPos[0])+sep+str(jPos[1])
                +sep+str(jPos[2])+sep+str(jPos[3])+sep+str(vac)+sep+str(speed)+endChar)
    return message

def waitForArduino():
    # Established serial connection
    while 1:
        var = s.dev.read().decode()
        print(f"{var, type(var)}")
        if var == startChar:
            s.write('A')
            print("Handshake Done!")
            s.dev.reset_input_buffer() # may be uneccessary
            data = s.read().strip()     # This line is essential and somehow reads the empty line that is output at the end of Setup
            break

def waitForResponse():
    while 1:
        if(s.dev.in_waiting > 0):
            #print(f"bytes:{s.dev.in_waiting}")
            data = s.read().strip()[1:-1] #gets data of form <I,435,45,4,3,3,2>
            #print(f"data: {data} type: {type(data)}")
            if not data: continue
            if (data[0] == info):
                # PARSE INFO INTO A LIST
                data_tup = tuple('('+data+')')
                infoList.append(data_tup)
                if(lastObjective == info):
                    print(data[2:])
                    return data[2:]
                    break
            
            if(data[0] == lastObjective):
                resultChar = data.split(splitChar)[-1]
                return(resultChar == successChar)
                break
        time.sleep(.05)
    return

def radToSteps(rad, microSteps, gearing = 3, stepsRev = 200):
    '''
    Conversion from radians to a number of steps 
    given ceratin steps per revolution and microstepping
    '''
    return rad/(2*math.pi) * stepsRev * microSteps * gearing

def stepsToRads(steps, microSteps, gearing =3,  stepsRev = 200):
    return steps/(stepsRev * microSteps *gearing) * (2*math.pi)

def worldToJoint(coords , thetab):
    '''
    Takes True world frame coordinates of the block! thetab is relative to block
    coords: tuple looks like -> (xb,yb,zb) 
    '''
    # First lets assume we are choosing the position of the end effector ie 45*
    roboBase = (0,0,85) 

    thetabRad = thetab

    L1 = 320    # mm  1st Arm length
    L2 = 320
    L3 = 0     # mm of end effector
    #TODO: Update these from CAD
    servoOffsetArm = 0#32 #mm offset from the joint of rotation
    servoOffsetZ = 0 #16 #mm offset from the joint of rotation

    suctionZoffset = 0#10 #Z offset from servo horn mm

    # Calculate the xyz of arm
    #First work backwords from block to position the end of servo
    za = coords[2] + suctionZoffset #Z_arm
    xa = coords[0] + L3*math.sin(thetabRad)
    ya = coords[1] - L3*math.cos(thetabRad)

    # Now calculate the position of the rotating joint of the arm
    za +=  servoOffsetZ

    # Time for Law of Cosines
    effectiveArmShadow = math.sqrt(xa**2 + ya**2)-servoOffsetArm # imagine shinging a light directly above the arm
    zextra = (za-roboBase[2]) # bottom of law of cosines trinagle
    a = L1
    c = L2
    b = math.sqrt(zextra**2 + effectiveArmShadow**2)
    
    gamma = math.atan2(zextra, effectiveArmShadow)

    # NOTE THETSE ARE BOTH Zero when they point straight out
    theta2 = math.acos((a**2 + b**2 - c**2)/(2*a*b)) + gamma
    theta3 = math.acos((a**2 - b**2 + c**2)/(2*a*c)) + theta2
    theta1 = math.atan2(ya, xa) - math.pi/2
    theta4 = thetabRad-theta1

    return (theta1, theta2, theta3, theta4)


def jointToWorld(stepPos):
    # COnstnats
    L1 = 320
    L2 = 320
    L3 = 0
    servoZOffset = 0#16
    servoArmOffset = 0 #  32
    eofZOffset = 0

    baseX = 0
    baseY = 0
    baseZ = 85

    jPos = stepTupleToRadTuple(stepPos)

    theta1 = jPos[0]
    theta2 = jPos[1]
    theta3 = jPos[2]   # Angle of stepper arm to the angle of the triangle
    theta4 = jPos[3]

    thetaI = theta3 - (math.pi/2)  # COuld be jPos - 90

    print(f"{theta1}, {theta2}, {theta3}, {theta4}, {thetaI}")
    middleJointZ = L1 * math.sin(theta2) - L2*math.cos(thetaI)
    middleJointArm = L1 * math.cos(theta2) + L2*math.sin(thetaI)

    # Now lets offset to servo, Please lets design this so the horn is in line
    servoZ = middleJointZ - servoZOffset
    servoArm = middleJointArm + servoArmOffset
    
    
    # Now lets decompose the arm
    servoHornX = servoArm * math.sin(-1*theta1)
    servoHornY = servoArm * math.cos(theta1)
    print(f"{baseX + servoHornX}, {baseY + servoHornY}, {baseZ + servoZ}")

    # now do the rotation of the end effector
    eofRot = theta1 + theta4
    eofX = L3 * math.sin(eofRot)
    eofY = L3 * math.cos(eofRot)
    
    # Apply EOF Z adjustment
    finalX = baseX + servoHornX - eofX
    finalY = baseY + servoHornY + eofY
    finalZ = baseZ + servoZ - eofZOffset 



    return (finalX,finalY,finalZ)
def radTupleToStepTuple(jPos):
    if len(jPos) != 4:
        print("Error!")
        return
    else:
        step1 = radToSteps(jPos[0], microSteps)
        step2 = radToSteps(jPos[1], microSteps)
        step3 = radToSteps(jPos[2], microSteps)
        step4 = jPos[3]
        return (int(step1), int(step2), int(step3), int(step4))

def stepTupleToRadTuple(stepPos):
    if len(stepPos) != 4:
        print("Error!")
        return
    rad1 = stepsToRads(stepPos[0], microSteps)
    rad2 = stepsToRads(stepPos[1], microSteps)
    rad3 = stepsToRads(stepPos[2], microSteps)
    rad4 = stepPos[3]
    return (rad1, rad2, rad3, rad4)

if __name__=='__main__':
    startChar = 'A'
    history = []   
    s = Serial_cmd()
    if not s.connected:
        print("Please Connect Arduino")
        quit()

    waitForArduino()
    
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
    theta1ZeroSteps = radToSteps(0, 32)
    theta2ZeroSteps = radToSteps(math.pi/2, 32)
    theta3ZeroSteps = radToSteps(math.pi, 32)
    homeTuple = (theta1ZeroSteps,theta2ZeroSteps, theta3ZeroSteps, 0)
    homingMessage = createMessage(home,homeTuple,0,0)
    s.write(homingMessage)
    print(f"Homing: {homingMessage}")
    result = waitForResponse()
    print(result)

    #Verify
    angle = 0
    block_angle = 0 + angle
    homeTuple = (theta1ZeroSteps,theta2ZeroSteps, theta3ZeroSteps, angle)
    print(f"HomeJointTuple: {homeTuple}")
    xyz = jointToWorld(homeTuple)
    print(f"World: {xyz}")
    jPos = worldToJoint(xyz, block_angle)
    print(f"Reconvert: {radTupleToStepTuple(jPos)}")



    # First move
    jPos = worldToJoint((0,450, 350), 0)
    stepPos = radTupleToStepTuple(jPos)
    nextPoint = createMessage(move,stepPos,1.0,40.0)
    s.write(nextPoint)
    print(f"sent message: {nextPoint}")
    result = waitForResponse()

    # First move
    jPos = worldToJoint((0,600, 100), 0)
    stepPos = radTupleToStepTuple(jPos)
    nextPoint = createMessage(move,stepPos,1.0,40.0)
    s.write(nextPoint)
    print(f"sent message: {nextPoint}")
    result = waitForResponse()

    #time.sleep(5)
    # Go home
    homingMessage = createMessage(move,homeTuple,0,0)
    s.write(homingMessage)
    print(f"Homing: {homingMessage}")
    result = waitForResponse()
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
