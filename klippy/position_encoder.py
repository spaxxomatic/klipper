# Serial port management for firmware communication
#
# Copyright (C) 2016-2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, threading
import serial

import msgproto, chelper, util

class error(Exception):
    pass

class Position:
    def __init__(self, reactor, serialport, baud):
        self.reactor = reactor
        self.serialport = serialport
        self.baud = baud
        # Serial port
        self.ser = None
        self.ffi_main, self.ffi_lib = chelper.get_ffi()
        self.serialqueue = None
        # Threading
        self.lock = threading.Lock()
        self.background_thread = None
    def _bg_thread(self):
        response = self.ffi_main.new('float get_x_pos')
        while 1:
            self.ffi_lib.serialqueue_pull(self.serialqueue, response)
            count = response.len
            if count <= 0:
                break
            params = self.msgparser.parse(response.msg[0:count])
            params['#sent_time'] = response.sent_time
            params['#receive_time'] = response.receive_time
    def _get_identify_data(self, timeout):
        # Query the "data dictionary" from the micro-controller
        identify_data = ""
        while 1:
            msg = "identify offset=%d count=%d" % (len(identify_data), 40)
            params = self.send_with_response(msg, 'identify_response')
            if params['offset'] == len(identify_data):
                msgdata = params['data']
                if not msgdata:
                    # Done
                    return identify_data
                identify_data += msgdata
            if self.reactor.monotonic() > timeout:
                raise error("Timeout during identify")
    def connect(self):
        # Initial connection
        logging.info("Connecting to encoders")
        start_time = self.reactor.monotonic()
        self.ffi.init_encoder_comm(self.port, self.baud)
        self.background_thread = threading.Thread(target=self._bg_thread)
        self.background_thread.start()
        msgparser = msgproto.MessageParser()
        msgparser.process_identify(identify_data)
        self.msgparser = msgparser
        self.register_response(self.handle_unknown, '#unknown')
        # Setup baud adjust
    
    def disconnect(self):
        if self.background_thread is not None:
            self.background_thread.join()
        self.ffi.shutdown_encoder()

    def __del__(self):
        self.disconnect()

# Class to retry sending of a query command until a given response is received
class SerialRetryCommand:
    TIMEOUT_TIME = 5.0
    RETRY_TIME = 0.500
    def __init__(self, serial, name, oid=None):
        self.serial = serial
        self.name = name
        self.oid = oid
        self.completion = serial.reactor.completion()
        self.min_query_time = serial.reactor.monotonic()
        self.serial.register_response(self.handle_callback, name, oid)
    def handle_callback(self, params):
        if params['#sent_time'] >= self.min_query_time:
            self.min_query_time = self.serial.reactor.NEVER
            self.serial.reactor.async_complete(self.completion, params)
    def get_response(self, cmds, cmd_queue, minclock=0, minsystime=0.):
        first_query_time = query_time = max(self.min_query_time, minsystime)
        while 1:
            for cmd in cmds:
                self.serial.raw_send(cmd, minclock, minclock, cmd_queue)
            params = self.completion.wait(query_time + self.RETRY_TIME)
            if params is not None:
                self.serial.register_response(None, self.name, self.oid)
                return params
            query_time = self.serial.reactor.monotonic()
            if query_time > first_query_time + self.TIMEOUT_TIME:
                self.serial.register_response(None, self.name, self.oid)
                raise error("Timeout on wait for '%s' response" % (self.name,))
