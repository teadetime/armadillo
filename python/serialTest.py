import serial
import serial.tools.list_ports as list_ports
import string, array
from datetime import datetime

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

    def get_yellow(self):
        if self.connected:
            self.write('YELLOW?')
            return int(self.read(), 16)

    def set_yellow(self, val):
        if self.connected:
            self.write('YELLOW!{:X}'.format(int(val)))


if __name__=='__main__':
    startChar = 'A'
    state = 0
    stop = False
    s = Serial_cmd()
    test_vals = []
    start = 0 
    while not stop:
        if state == 0:
            ## Established serial connection
            var = s.dev.read().decode()
            print(f"{var, type(var)}")
            if var == startChar:
                s.write('A')
                print("Handshake Done!")
                state = 1
                start = datetime.now()
                continue
        if state == 1:  
            data = s.read().strip()[1:-1]
            #print(data)
            test_vals.append(data)
            if len(test_vals) >= 1000:
                stop = True
    print(f"first:{test_vals[0]} last: {test_vals[-1]}, Count:{len(test_vals)}")
    seconds = datetime.now()-start
    seconds = seconds.total_seconds()
    print(f"total time: {seconds}, Hz: {len(test_vals)/seconds}")