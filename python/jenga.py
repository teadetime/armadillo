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
                    return data[2:]
                    break
            
            if(data[0] == lastObjective):
                resultChar = data.split(splitChar)[-1]
                return(resultChar == successChar)
                break
        time.sleep(.05)
    return

def radToSteps(rad, microSteps, stepsRev = 200):
    '''
    Conversion from radians to a number of steps 
    given ceratin steps per revolution and microstepping
    '''
    return rad/(2*math.pi) * stepsRev * microSteps

def worldToJoint( coords , thetab):
    '''
    Takes True world frame coordinates of the block! thetab is relative to block
    coords: tuple looks like -> (xb,yb,zb) 
    '''
    # First lets assume we are choosing the position of the end effector ie 45*
    roboBase = (0,0,95) 

    thetabRad = math.radians(thetab)

    L1 = 180    # mm  1st Arm length
    L2 = 180
    L3 = 0     # mm of end effector
    #TODO: Update these from CAD
    servoOffsetArm = 30 #mm offset from the joint of rotation
    servoOffsetZ = 15 #mm offset from the joint of rotation

    suctionZoffset = 0 #Z offset from servo horn mm

    # Calculate the xyz of arm
    #First work backwords from block to position the end of servo
    za = coords[2] + suctionZoffset #Z_arm
    xa = coords[0] + L3*math.sin(thetabRad)
    ya = coords[1] - L3*math.cos(math.radians(thetabRad))

    # Now calculate the position of the rotating joint of the arm
    za -=  servoOffsetZ

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
    theta4 = theta1 + thetabRad

    return (theta1, theta2, theta3, theta4)

def radTupleToSteptuple(jPos):
    if len(jPos) != 4:
        print("Error!")
        return
    else:
        step1 = radToSteps(jPos[0], microSteps)
        step2 = radToSteps(jPos[1], microSteps)
        step3 = radToSteps(jPos[2], microSteps)
        step4 = 0
        return (step1, step2, step3, step4)

if __name__=='__main__':
    startChar = 'A'
    state = 1
    stop = False
    test_vals = []
    
    history = []
    messToSend = 5
    counter = 0

   
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
    last_send_time = datetime.now()
    #for jengaBlock in range(54):
    theta1ZeroSteps = radToSteps(0, 32)
    theta2ZeroSteps = radToSteps(math.pi/2, 32)
    theta3ZeroSteps = radToSteps(3*math.pi/4, 32)
    homeTuple = (theta1ZeroSteps,theta2ZeroSteps, theta3ZeroSteps, 0)
    homingMessage = createMessage(home,homeTuple,0,0)
    s.write(homingMessage)
    print(f"Homing: {homingMessage}")
    result = waitForResponse()
    print(result)

    test = createMessage(info,(0,0,0,0),0,0)
    s.write(test)
    print(f"Test: {test}, {lastObjective}")
    result = waitForResponse()
    print(result)



    # Go to a position
    jPos = worldToJoint((0,220, 120), 0)
    stepPos = radTupleToSteptuple(jPos)
    nextPoint = createMessage(move,(0,0,0,0),1.0,40.0)
    s.write(nextPoint)
    print(f"sent message: {nextPoint}")
    result = waitForResponse()
    print(result)


    nextPoint = createMessage(move,(0,1600,2400,0),1.0,40.0)
    s.write(nextPoint)
    print(f"sent message: {nextPoint}")
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
