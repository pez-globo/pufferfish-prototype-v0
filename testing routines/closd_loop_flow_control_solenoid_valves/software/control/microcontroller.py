import platform
import serial
import serial.tools.list_ports
import time
import numpy as np

from control._def import *

# add user to the dialout group to avoid the need to use sudo

class Microcontroller():
    def __init__(self,parent=None):
        self.serial = None
        self.platform_name = platform.system()
        self.tx_buffer_length = MicrocontrollerDef.CMD_LENGTH
        self.rx_buffer_length = MicrocontrollerDef.MSG_LENGTH

        # AUTO-DETECT the Arduino! By Deepak
        arduino_ports = [
                p.device
                for p in serial.tools.list_ports.comports()
                if 'Arduino' in p.description]
        if not arduino_ports:
            raise IOError("No Arduino found")
        if len(arduino_ports) > 1:
            print('Multiple Arduinos found - using the first')
        else:
            print('Using Arduino found at : {}'.format(arduino_ports[0]))

        # establish serial communication
        self.serial = serial.Serial(arduino_ports[0],2000000)
        time.sleep(0.2)
        print('Serial Connection Open')

    def close(self):
        self.serial.close()

    def toggle_LED(self,state):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = 3
        cmd[1] = state
        self.serial.write(cmd)
    
    def toggle_laser(self,state):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = 4
        cmd[1] = state
        self.serial.write(cmd)

    def move_x(self,delta):
        direction = int((np.sign(delta)+1)/2)
        n_microsteps = abs(delta*Motion.STEPS_PER_MM_XY)
        if n_microsteps > 65535:
            n_microsteps = 65535
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = 0
        cmd[1] = direction
        cmd[2] = int(n_microsteps) >> 8
        cmd[3] = int(n_microsteps) & 0xff
        self.serial.write(cmd)
        time.sleep(WaitTime.BASE + WaitTime.X*abs(delta))

    def cycle_x(self):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = 4
        cmd[1] = 0
        self.serial.write(cmd)
        print('trying to cycle x')

    def send_command(self,cmd):
        self.serial.write(cmd)

    def read_received_packet(self):
        # wait to receive data
        while self.serial.in_waiting==0:
            pass
        while self.serial.in_waiting % self.rx_buffer_length != 0:
            pass

        num_bytes_in_rx_buffer = self.serial.in_waiting

        # get rid of old data
        if num_bytes_in_rx_buffer > self.rx_buffer_length:
            print('getting rid of old data')
            for i in range(num_bytes_in_rx_buffer-self.rx_buffer_length):
                self.serial.read()
        
        # read the buffer
        data=[]
        for i in range(self.rx_buffer_length):
            data.append(ord(self.serial.read()))

        '''
        YfocusPhase = self.data2byte_to_int(data[0],data[1])*2*np.pi/65535.
        Xpos_arduino = data[3]*2**24 + data[4]*2**16+data[5]*2**8 + data[6]
        if data[2]==1:
            Xpos_arduino =-Xpos_arduino
        Ypos_arduino = data[8]*2**24 + data[9]*2**16+data[10]*2**8 + data[11]
        if data[7]==1:
            Ypos_arduino =-Ypos_arduino
        Zpos_arduino = data[13]*2**24 + data[14]*2**16+data[15]*2**8 + data[16]
        if data[12]==1:
            Zpos_arduino =-Zpos_arduino
        manualMode = data[17]
        LED_measured = self.data2byte_to_int(data[18], data[19])
        timeStamp = data[20]*2**24 + data[21]*2**16+data[22]*2**8 + data[23]
        tracking_triggered = bool(data[24])
        trigger_FL = bool(data[25])
        return [YfocusPhase,Xpos_arduino,Ypos_arduino,Zpos_arduino, LED_measured, tracking_triggered],manualMode
        '''
        return data

    def read_received_packet_nowait(self):
        # wait to receive data
        if self.serial.in_waiting==0:
            return None
        if self.serial.in_waiting % self.rx_buffer_length != 0:
            print(str(self.serial.in_waiting) + '/' + str(self.rx_buffer_length))
            num_bytes_in_rx_buffer = self.serial.in_waiting
            for i in range(num_bytes_in_rx_buffer):
                self.serial.read()
            print('reset input buffer')
            return None
        
        # get rid of old data
        num_bytes_in_rx_buffer = self.serial.in_waiting
        if num_bytes_in_rx_buffer > self.rx_buffer_length:
            print('getting rid of old data')
            for i in range(num_bytes_in_rx_buffer-self.rx_buffer_length):
                self.serial.read()
        
        # read the buffer
        data=[]
        for i in range(self.rx_buffer_length):
            data.append(ord(self.serial.read()))
        return data

class Microcontroller_Simulation():
    def __init__(self,parent=None):
        pass

    def close(self):
        pass

    def toggle_LED(self,state):
        pass
    
    def toggle_laser(self,state):
        pass

    def move_x(self,delta):
        pass

    def move_y(self,delta):
        pass

    def move_z(self,delta):
        pass

    def send_command(self,command):
        pass

    def read_received_packet(self):
        pass

    def read_received_packet_nowait(self):
        return None


# from Gravity machine
def split_int_2byte(number):
    return int(number)% 256,int(number) >> 8

def split_signed_int_2byte(number):
    if abs(number) > 32767:
        number = np.sign(number)*32767

    if number!=abs(number):
        number=65536+number
    return int(number)% 256,int(number) >> 8

def split_int_3byte(number):
    return int(number)%256, int(number) >> 8, int(number) >> 16

def data2byte_to_int(a,b):
    return a + 256*b

def data2byte_to_signed_int(a,b):
    nb= a+256*b
    if nb>32767:
        nb=nb-65536
    return nb

def data4byte_to_int(a,b,c,d):
    return a + (256)*b + (65536)*c + (16777216)*d
