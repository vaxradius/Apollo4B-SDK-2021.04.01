import serial
import serial.tools.list_ports
import datetime
import random
import signal
#import numpy as data_array
#import os

from time import sleep

import sys

poly32 = 0x1EDC6F41

CDC_DATA_IN_BUFFER_SIZE = (64*1024)

def py_crc32(L):
    rem = 0
    for b in L:
        rem = rem ^ (b << 24)
        for i in range(8):
            if rem & 0x80000000:
                rem = ((rem << 1) ^ poly32)
            else:
                rem = (rem << 1)

            rem = rem & 0xFFFFFFFF
    return rem
 
 
class Logger(object):
    def __init__(self, filename='LOG_default.txt', stream=sys.stdout):
        self.terminal = stream
        self.log = open(filename, 'w')
 
    def write(self, message):
        self.terminal.write(message)
        self.log.write(message)
 
    def flush(self):
        pass
   
#sys.stdout = Logger('LOG_cdc_random_tet.txt', sys.stdout)
#sys.stderr = Logger('Error_cdc_random_tet.txt', sys.stderr)

num_sample = [1, 10, 100, 1000]
chars_sample = 'abcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXYZ'

data_out_1 = '0123456789ABCDEFGH0123456789ABCDEFGH0123456789ABCDEFGH0123456789ABCDEFGH0123456789ABCDEFGH0123456789ABCDEFGH0123456789ABCDEFGH0123456789ABCDEFGH0123456789ABCDEFGH0123456789ABCDEFGH0123456789ABCDEFGH0123456789ABCDEFGH'\
       '0123456789ABCDEFGH0123456789ABCDEFGH0123456789ABCDEFGH0123456789ABCDEFGH0123456789ABCDEFGH0123456789ABCDEFGH0123456789ABCDEFGH0123456789ABCDEFGH0123456789ABCDEFGH0123456789ABCDEFGH0123456789ABCDEFGH0123456789ABCDEFGH'\
       '0123456789ABCDEFGH0123456789ABCDEFGH0123456789ABCDEFGH0123456789ABCDEFGH0123456789ABCDEFGH0123456789ABCDEFGH0123456789ABCDEFGH0123456789ABCDEFGH0123456789ABCDEFGH0123456789ABCDEFGH0123456789ABCDEFGH0123456789ABCDEFGH'.encode()

one2nine = 0

def gen_random_str():
    
    dataout = ''
    
    thousands_byte = '0'
    hundreds_byte = '0'
    tens_byte = '0'
    digits_byte = '0'
    
    seed = random.randint(1,9999)
    
    print('-- random seed = ' + str(seed))

    num = str(seed)
    lens = len(num)

    if(lens >= 4):
        thousands_byte = num[lens-4:lens-3]
    else:
        thousands_byte = '0'

    if (lens >= 3):
        hundreds_byte  = num[lens-3:lens-2]
    else:
        hundreds_byte  = '0'

    if (lens >= 2):        
        tens_byte      = num[lens-2:lens-1]
    else:
        tens_byte      = '0'
        
    if (lens >= 1):
        digits_byte    = num[lens-1:lens]
        
    if int(digits_byte) > 0:
        data_random = random.sample(chars_sample, 1)
        for i in range(int(digits_byte)):
            dataout += ''.join(data_random)
            
    if int(tens_byte) > 0:
        data_random = random.sample(chars_sample, 10)
        for i in range(int(tens_byte)): 
            dataout += ''.join(data_random)
            
    if int(hundreds_byte) > 0:
        data_random = random.sample(chars_sample, 20)
        for i in range(5*int(hundreds_byte)):
            dataout += ''.join(data_random)
            
    if int(thousands_byte) > 0:
        data_random = random.sample(chars_sample, 40)
        for i in range(25*int(thousands_byte)):
            dataout += ''.join(data_random)
    #else:
    #    data_random = random.sample(chars_sample, 50)
    #    for i in range(200*seed):
    #        dataout += ''.join(data_random)
  
    return dataout


def gen_increment_str(test_data_len_counter):

    dataout = ''    
    lens = test_data_len_counter    #len(num)

    dataout = data_array.empty(test_data_len_counter, dtype = data_array.uint8).tobytes()
    #print(dataout)
    return dataout

def recv(serial, length):
    while True:
        try:
            #data_serial = serial.read_all()
            data_serial = serial.read(length)
            if data_serial == b'':
            #if data_serial == '':
                continue
            else:
                break
            sleep(0.02)
        except KeyboardInterrupt:
            serial.close()
    return data_serial

def handler(signal_num,frame):
   print("Serial port closed.")
   mySerial.close()
   sys.exit(signal_num)

signal.signal(signal.SIGINT, handler)

if __name__ == '__main__':
    #pid="0483"  #STMicro PID HWID
    #hid="5740"
    pid="4001"  #Apollo4 test PID HWID
    hid="CAFE"
    my_comm_port = None
    dongle_found = False
    dongle_connected = False
    loop_count = 0
    progress_icon = "-\\|/"
    process_ok = True

    print("===========================================================")
    print("Random Data Loop Back Test")
    print("Ambiq Micro\nwww.ambiqmicro.com")
    print("Version 1.1, 2020-06-18")
    print("===========================================================")

    # load apollo4 binary via jlink
    #print("Loading Apollo4 test firmware...", end="\r", flush=True)
    #os.system('jlink -CommanderScript ap4_tinyusb_cdc_loopback_example.txt')
    print('Done')
    print('')
    while dongle_connected == False:
        while dongle_found == False:
            print("Looking for Ambiq FS Dongle: ", progress_icon[loop_count%4], end="\r", flush=True)
            loop_count+=1
            ports = list(serial.tools.list_ports.comports())
            for p in ports:
                if pid and hid in p.hwid:
                    my_comm_port = p.device
                    dongle_found = True
                    print("")   # new line
                    print("-- Ambiq FS Dongle found on", my_comm_port)
            if my_comm_port == None:
                if (loop_count % 50 == 0):
                    print("")   # new line
            sleep(1)            # sleep for 1 second

        try:
            mySerial = serial.Serial(my_comm_port, 115200, timeout=None)    # baudrate = 0 as the vDFU enter command
            sleep(0.5)          # give it some time before open
            mySerial.isOpen()
            mySerial.flush()
            dongle_connected = True
            print("-- Connected.")
        except :
            print("-- Failed to open com port on dongle. Please try again.")
            dongle_connected = False
            dongle_found = False
            loop_count = 0
            my_comm_port = None   
    iteration_counter = 0
    failure_counter = 0
    test_data_len_counter = 0;
    while True:
        try:
            data_out = gen_random_str().encode()
            #test_data_len_counter = test_data_len_counter + 1
            #data_out = gen_increment_str(test_data_len_counter)
            #if test_data_len_counter >= 9999:
            #    test_data_len_counter = 1;
            
            print('-- data length = ' + str(len(data_out)))
            #sleep(0.1)
            mySerial.flush()
            mySerial.write(data_out)
            #print(str(data_out))
            #sleep(0.4)
            dataRev = recv(mySerial, len(data_out))

            curtime = datetime.datetime.now().strftime('%H:%M:%S----------->')
            if(dataRev == data_out):
                print(curtime+"success: loopback")
            else:
                print(curtime+"fail: no data loopback")
                failure_counter = failure_counter + 1
            iteration_counter = iteration_counter + 1
            print('-- interation:' + str(iteration_counter))
            print('-- failure: ' + str(failure_counter))
        except KeyboardInterrupt:
            mySerial.close()
            sleep(0.5)
            print("Test done at", curtime)
            print('-- interation:' + str(iteration_counter))
            print('-- failure: ' + str(failure_counter))
            break
    while True:
        sleep(5)
            
    
#        data =recv(serial)
#        if data != b'' :
#            print("receive : ",data)
