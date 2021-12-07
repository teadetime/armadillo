import serial
import serial.tools.list_ports as list_ports
import string, array
from datetime import datetime
import time

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

def createMessage(jPos, vac, speed, startChar = '<', endChar = '>', sep = ','):
    """
    Sends form startChar j1,j2,j3,j4,vac,speed endChar
    ie. <0,20,30,50,1.0,50>
    """
    # Make sure that jPos etc are trimmed to x decimals
    # Should we send angles or steps?
    message = (startChar+str(jPos[0])+sep+str(jPos[1])
                +sep+str(jPos[2])+sep+str(jPos[3])+sep+str(vac)+sep+str(speed)+endChar)
    return message

def sendMessage(message):
    s.write(message)
    return message

def worldToJoint(xyzPos):
    jPos = xyzPos
    return jPos

def waitForArduino():
    # Established serial connection
    while 1:
        var = s.dev.read().decode()
        print(f"{var, type(var)}")
        if var == startChar:
            s.write('A')
            print("Handshake Done!")
            s.dev.reset_input_buffer() # may be uneccessary
            # state = 1
            # start = datetime.now()
            break


if __name__=='__main__':
    startChar = 'A'
    state = 1
    stop = False
    test_vals = []
    
    history = []
    messToSend = 5
    counter = 0

    last_send_time = datetime.now()
    messageSpacing = 1
    s = Serial_cmd()
    if not s.connected:
        print("Please Connect Arduino")
        quit()

    waitForArduino()
    start = datetime.now()
    while not stop:
        if state == 1:  
            data = s.read().strip()#[1:-1]

            seconds = datetime.now()-last_send_time


            if seconds.total_seconds() > messageSpacing:
                # Send a message
                jPos = (0,1,2.0,3)
                nextPoint = createMessage(jPos,1.0,40.0)
                sendMessage(nextPoint)
                history.append((nextPoint, datetime.now))
                last_send_time = datetime.now()
            print(data)
            test_vals.append(data)

            if len(test_vals) >= 500:
                state = 2
                stop = True
                print(f"first:{test_vals[0]} last: {test_vals[-1]}, Count:{len(test_vals)}")
                seconds = datetime.now()-start
                seconds = seconds.total_seconds()
                print(f"total time: {seconds}, Hz: {len(test_vals)/seconds}")

