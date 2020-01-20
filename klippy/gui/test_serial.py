import serial
import ConfigParser
from thread import start_new_thread
import time
import dromacros as macros
import os
import sys
import struct
myfolder = os.path.dirname(os.path.realpath(__file__))

configfile = os.path.join(myfolder,"dro.ini")
os.chdir(myfolder)
print "Reading config file %s"%configfile
config = ConfigParser.ConfigParser()
config.read(configfile)
port, baudrate = None, None
try:
    port = config.get('GENERAL', 'comport')    
    baudrate = config.get('GENERAL', 'baudrate')    
except Exception, e:
    print "Invalid config file: %s"%(str(e))
    exit(1)

class SerialConn():
    buff = []
    def __init__(self):
        self.con = serial.Serial(
            port=port, \
            baudrate=baudrate, \
            parity=serial.PARITY_NONE, \
            stopbits=serial.STOPBITS_ONE, \
            bytesize=serial.EIGHTBITS, \
            timeout=None)

    def receive(self):
        print( '..init..')
        cnt = 0
        while True:
            bytes = self.con.read(self.con.in_waiting or 1)
            if bytes :
                sys.stdout.write (byte)

    def receive_line(self):
        print( '..init..')
        cnt = 0
        while True:
            #print self.con.in_waiting
            #data = self.con.read(self.con.in_waiting or 1)
            data = self.con.readline()
            if data:
                lines = data.split('\n')
                #print lines
                #line = self.con.read_until('\n')
                #print (len(lines))
                for line in lines:
                    if len(line) == 3:
                        axis = line[0]
                        if line[1] == '\xff':
                            pass
                            #sys.stdout.write (axis)
                            #sys.stdout.write ('ERR')
                            #print (int(line[2].encode('hex'), 8))
                        else:
                            sys.stdout.write (axis)
                            enc_val = struct.unpack('>H', line[1:3])[0]
                            #enc_val = int(line[1:3].encode('hex'), 16)
                            print str(enc_val)
            #sys.stdout.write (str(len(line)))
            #if (cnt == 100):
            #    cnt = 0
            #    print("")


msg = " "
threadmethod = None
print('Opening DRO port ' + port + ' at ' + baudrate)
SerialConn().receive_line()

