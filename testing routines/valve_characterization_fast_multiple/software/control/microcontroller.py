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

        print('Rec buffer size: {}'.format(self.rx_buffer_length))

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

    def move_valve(self, delta):

        print('Trying to move valve by {}'.format(delta))

        if(delta>=0):
            direction = 1;
        else:
            direction = 0;
        # direction = int((np.sign(delta)+1)/2)

        n_microsteps = abs(delta*Motion.STEPS_PER_MM)
        if n_microsteps > 65535:
            n_microsteps = 65535

        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = 2
        cmd[1] = direction
        cmd[2] = int(n_microsteps) >> 8
        cmd[3] = int(n_microsteps) & 0xff

        self.serial.write(cmd)
        # time.sleep(WaitTime.BASE + WaitTime.X*abs(delta))
        print('finished move')

    def close_valve(self):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = 3
        cmd[1] = 0
        self.serial.write(cmd)
        print('trying to close valve')

    def start_cycle_valve(self):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = 4
        cmd[1] = 1
        self.serial.write(cmd)
        print('start cycling all valves')

    def stop_cycle_valve(self):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = 4
        cmd[1] = 0
        self.serial.write(cmd)
        print('stop cycling all valves')

    def update_active_valve(self, valve_id):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = 5
        cmd[1] = 0
        cmd[2] = valve_id

        self.serial.write(cmd)
        print('set active valve to: {}'.format(valve_id))

    def enable_valve_measurement_cycling(self, cycles_per_valve):

        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = 5
        cmd[1] = 1
        cmd[2] = int(cycles_per_valve) >> 8
        cmd[3] = int(cycles_per_valve) & 0xff

        self.serial.write(cmd)
        print('Enabled cycling measurements of valve at {} cycles/valve'.format(cycles_per_valve))


    def send_command(self,command):
        cmd = bytearray(self.tx_buffer_length)
        '''
        cmd[0],cmd[1] = self.split_int_2byte(round(command[0]*100))                #liquid_lens_freq
        # cmd[2],cmd[3]=self.split_int_2byte(round(command[1]*1000))               #liquid_lens_ampl
        # cmd[4],cmd[5]=self.split_int_2byte(round(command[2]*100))                #liquidLens_offset
        cmd[2] = int(command[1])                                                   # Focus-Tracking ON or OFF
        cmd[3] = int(command[2])                                                   #Homing
        cmd[4] = int(command[3])                                                   #tracking
        cmd[5],cmd[6] = self.split_signed_int_2byte(round(command[4]*100))         #Xerror
        cmd[7],cmd[8] = self.split_signed_int_2byte(round(command[5]*100))         #Yerror                           
        cmd[9],cmd[10] = self.split_signed_int_2byte(round(command[6]*100))        #Zerror
        cmd[11],cmd[12] = self.split_int_2byte(round(0))#command[9]*10))               #averageDt (millisecond with two digit after coma) BUG
        cmd[13] = int(command[8])                                               # LED intensity
        # Adding Trigger flag for other Video Streams (Boolean)
        # print('Trigger command sent {}'.format(command[9]))
        cmd[14] = int(command[9])
        # Adding Sampling Interval for other Video Streams
        # Minimum 10 ms (0.01 s) Maximum: 3600 s (1 hour)
        # Min value: 1 to 360000 
        # print('Interval command sent {}'.format(command[10]))
        cmd[15], cmd[16] = self.split_int_2byte(round(100*command[10]))
        '''
        self.serial.write(cmd)


    def read_received_packet_nowait(self):
        num_bytes_in_rx_buffer = self.serial.in_waiting
        # print(num_bytes_in_rx_buffer)
        # wait to receive data
        if num_bytes_in_rx_buffer == 0:
            # print('Waiting for data')
            return None

        # get rid of old data
        num_bytes_in_rx_buffer = self.serial.in_waiting
        if num_bytes_in_rx_buffer > self.rx_buffer_length:
            print('getting rid of old data')
            for i in range(num_bytes_in_rx_buffer-self.rx_buffer_length):
                self.serial.read()

        num_bytes_in_rx_buffer = self.serial.in_waiting
        if num_bytes_in_rx_buffer % self.rx_buffer_length != 0:
            # if(num_bytes_in_rx_buffer == 1020):
            #     print('clearing buffer')
            #     for i in range(num_bytes_in_rx_buffer):
            #         self.serial.read()
            #     return None
            # else:
            print('Buffer not full')
            return None



        
        
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
