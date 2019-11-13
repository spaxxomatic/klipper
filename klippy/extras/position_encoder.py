#  Support for linear encoders connected to the serial port
#
# Copyright (C) 2016-2019  Lucian Nutiu <lucian.nutiu@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, threading
import serial
import chelper
import time
class error(Exception):
    pass

message_encoder_connect_error = '''Encoder connection could not be established. The encoder must 
be connected to the serial port of the Raspi board. Please check your config. '''

class PositionFeedback:   
    def __init__(self, config):
        self.printer = config.get_printer()
        self.encoder_serial_port = config.get('encoder_serial_port')
        self.baud = config.getint('encoder_baud')
        self.printer.register_event_handler("klippy:ready", self.connect)
        self.printer.register_event_handler("klippy:disconnect", self.disconnect)
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
            axis_stat = self.ffi_lib.get_axis_stat()
            #print ("X POS " + str(x_pos))
            time.sleep(1)
    def connect(self):
        # Initial connection
        logging.info("Connecting encoders")
        #start_time = self.reactor.monotonic()
        self.ffi_lib.init_encoder_comm(self.encoder_serial_port, self.baud)
        self.background_thread = threading.Thread(target=self._bg_thread)
        self.background_thread.start()
        
    def disconnect(self):
        if self.background_thread is not None:
            self.background_thread.join()
        self.ffi.shutdown_encoder()

    def __del__(self):
        self.disconnect()

def load_config_prefix(config):
    return PositionFeedback(config)
