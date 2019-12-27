#  Support for linear encoders connected to the serial port
#
# Copyright (C) 2016-2019  Lucian Nutiu <lucian.nutiu@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, threading
import serial
import chelper
import time
import serial
class error(Exception):
    pass

message_encoder_connect_error = '''Encoder connection could not be established. The encoder must 
be connected to the serial port of the Raspi board. Please check your config. '''

class PositionFeedback:   
    def __init__(self, config):
        self.printer = config.get_printer()
        self.encoder_serial_port = config.get('encoder_serial_port')
        self.scales_baud = config.getint('encoder_baud')
        self.printer.register_event_handler("klippy:ready", self.connect)
        self.printer.register_event_handler("klippy:disconnect", self.disconnect)
        self.scales_fd = None        

        # Serial port
        self.ser = None
        self.ffi_main, self.ffi_lib = chelper.get_ffi()
        self.get_x_pos = self.ffi_lib.get_x_pos
        self.get_y_pos = self.ffi_lib.get_y_pos
        # Threading
        self.lock = threading.Lock()
        self.background_thread = None
    def _bg_thread(self):
        #self.axis_stat = 
        while 1:
            #axis_stat = self.ffi_lib.get_axis_stat()
            x_pos = self.ffi_lib.get_x_pos()
            y_pos = self.ffi_lib.get_y_pos()
            print ("POS X:%f Y:%f"%(x_pos, y_pos))
            time.sleep(2)
    def connect(self):
        # Initial connection
        logging.info("Connecting encoders")
        try:
            self.scales_fd = serial.Serial(
                self.encoder_serial_port, self.scales_baud, timeout=0, exclusive=True)
            #else:
            #    self.scales_fd = open(self.scales_serial_port, 'rb+')
        except (OSError, IOError, serial.SerialException) as e:
            logging.error("Unable to connect encoder via port %s:"%self.encoder_serial_port ,e)
        self.serialqueue = self.ffi_lib.get_serialqueue()
        self.ffi_lib.init_encoder_poll(self.serialqueue, self.scales_fd.fileno())
        return self.scales_fd

        
    def disconnect(self):
        if self.background_thread is not None:
            self.background_thread.join()
        if self.scales_fd is not None:
            self.scales_fd.close()
            self.scales_fd = None

    def __del__(self):
        self.disconnect()

def load_config_prefix(config):
    return PositionFeedback(config)
