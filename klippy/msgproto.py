# Protocol definitions for firmware communication
#
# Copyright (C) 2016-2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import json, zlib, logging

DefaultMessages = {
    "identify_response offset=%u data=%.*s": 0,
    "identify offset=%u count=%c": 1,
}

MESSAGE_MIN = 5
MESSAGE_MAX = 64
MESSAGE_HEADER_SIZE  = 2
MESSAGE_TRAILER_SIZE = 3
MESSAGE_POS_LEN = 0
MESSAGE_POS_SEQ = 1
MESSAGE_TRAILER_CRC  = 3
MESSAGE_TRAILER_SYNC = 1
MESSAGE_PAYLOAD_MAX = MESSAGE_MAX - MESSAGE_MIN
MESSAGE_SEQ_MASK = 0x0f
MESSAGE_DEST = 0x10
MESSAGE_SYNC = '\x7E'
MESSAGE_ESCAPE = '\xFF'
NULL_BYTE_MASTER = '\xFD'

class error(Exception):
    pass

def crc16_ccitt(buf):
    crc = 0xffff
    for data in buf:
        data = ord(data)
        data ^= crc & 0xff
        data ^= (data & 0x0f) << 4
        crc = ((data << 8) | (crc >> 8)) ^ (data >> 4) ^ (data << 3)
    crc = chr(crc >> 8) + chr(crc & 0xff)
    return crc

class PT_uint32:
    is_int = True
    is_dynamic_string = False
    max_length = 5
    signed = False
    def encode(self, out, v):
        if v >= 0xc000000 or v < -0x4000000: out.append((v>>28) & 0x7f | 0x80)
        if v >= 0x180000 or v < -0x80000:    out.append((v>>21) & 0x7f | 0x80)
        if v >= 0x3000 or v < -0x1000:       out.append((v>>14) & 0x7f | 0x80)
        if v >= 0x60 or v < -0x20:           out.append((v>>7)  & 0x7f | 0x80)
        out.append(v & 0x7f)
    def parse(self, s, pos):
        c = s[pos]
        pos += 1
        v = c & 0x7f
        if (c & 0x60) == 0x60:
            v |= -0x20
        while c & 0x80:
            c = s[pos]
            pos += 1
            v = (v<<7) | (c & 0x7f)
        if not self.signed:
            v = int(v & 0xffffffff)
        return v, pos

class PT_int32(PT_uint32):
    signed = True
class PT_uint16(PT_uint32):
    max_length = 3
class PT_int16(PT_int32):
    signed = True
    max_length = 3
class PT_byte(PT_uint32):
    max_length = 2

class PT_string:
    is_int = False
    is_dynamic_string = True
    max_length = 64
    def encode(self, out, v):
        out.append(len(v))
        out.extend(bytearray(v))
    def parse(self, s, pos):
        l = s[pos]
        return str(bytearray(s[pos+1:pos+l+1])), pos+l+1
class PT_progmem_buffer(PT_string):
    pass
class PT_buffer(PT_string):
    pass

MessageTypes = {
    '%u': PT_uint32(), '%i': PT_int32(),
    '%hu': PT_uint16(), '%hi': PT_int16(),
    '%c': PT_byte(),
    '%s': PT_string(), '%.*s': PT_progmem_buffer(), '%*s': PT_buffer(),
}

class Enumeration:
    is_int = False
    is_dynamic_string = False
    def __init__(self, pt, enum_name, enums):
        self.pt = pt
        self.max_length = pt.max_length
        self.enum_name = enum_name
        self.enums = enums
        self.reverse_enums = {v: k for k, v in enums.items()}
    def encode(self, out, v):
        tv = self.enums.get(v)
        if tv is None:
            raise error("Unknown value '%s' in enumeration '%s'" % (
                v, self.enum_name))
        self.pt.encode(out, tv)
    def parse(self, s, pos):
        v, pos = self.pt.parse(s, pos)
        tv = self.reverse_enums.get(v)
        if tv is None:
            tv = "?%d" % (v,)
        return tv, pos

# Lookup the message types for a format string
def lookup_params(msgformat, enumerations={}):
    out = []
    argparts = [arg.split('=') for arg in msgformat.split()[1:]]
    for name, fmt in argparts:
        pt = MessageTypes[fmt]
        for enum_name, enums in enumerations.items():
            if name == enum_name or name.endswith('_' + enum_name):
                pt = Enumeration(pt, enum_name, enums)
                break
        out.append((name, pt))
    return out

# Update the message format to be compatible with python's % operator
def convert_msg_format(msgformat):
    for c in ['%u', '%i', '%hu', '%hi', '%c', '%.*s', '%*s']:
        msgformat = msgformat.replace(c, '%s')
    return msgformat

class MessageFormat:
    def __init__(self, msgid, msgformat, enumerations={}):
        self.msgid = msgid
        self.msgformat = msgformat
        self.debugformat = convert_msg_format(msgformat)
        self.name = msgformat.split()[0]
        self.param_names = lookup_params(msgformat, enumerations)
        self.param_types = [t for name, t in self.param_names]
        self.name_to_type = dict(self.param_names)
    def encode(self, params):
        out = []
        out.append(self.msgid)
        for i, t in enumerate(self.param_types):
            t.encode(out, params[i])
        return out
    def encode_by_name(self, **params):
        out = []
        out.append(self.msgid)
        for name, t in self.param_names:
            t.encode(out, params[name])
        return out
    def parse(self, s, pos):
        pos += 1
        out = {}
        for name, t in self.param_names:
            v, pos = t.parse(s, pos)
            out[name] = v
        return out, pos
    def format_params(self, params):
        out = []
        for name, t in self.param_names:
            v = params[name]
            if t.is_dynamic_string:
                v = repr(v)
            out.append(v)
        return self.debugformat % tuple(out)

class OutputFormat:
    name = '#output'
    def __init__(self, msgid, msgformat):
        self.msgid = msgid
        self.msgformat = msgformat
        self.debugformat = convert_msg_format(msgformat)
        self.param_types = []
        args = msgformat
        while 1:
            pos = args.find('%')
            if pos < 0:
                break
            if pos+1 >= len(args) or args[pos+1] != '%':
                for i in range(4):
                    t = MessageTypes.get(args[pos:pos+1+i])
                    if t is not None:
                        self.param_types.append(t)
                        break
                else:
                    raise error("Invalid output format for '%s'" % (msg,))
            args = args[pos+1:]
    def parse(self, s, pos):
        pos += 1
        out = []
        for t in self.param_types:
            v, pos = t.parse(s, pos)
            if t.is_dynamic_string:
                v = repr(v)
            out.append(v)
        outmsg = self.debugformat % tuple(out)
        return {'#msg': outmsg}, pos
    def format_params(self, params):
        return "#output %s" % (params['#msg'],)

class UnknownFormat:
    name = '#unknown'
    def parse(self, s, pos):
        msgid = s[pos]
        msg = str(bytearray(s))
        return {'#msgid': msgid, '#msg': msg}, len(s)-MESSAGE_TRAILER_SIZE
    def format_params(self, params):
        return "#unknown %s" % (repr(params['#msg']),)

class MessageParser:
    error = error
    def __init__(self):
        self.unknown = UnknownFormat()
        self.enumerations = {}
        self.command_ids = []
        self.messages_by_id = {}
        self.messages_by_name = {}
        self.config = {}
        self.version = self.build_versions = ""
        self.raw_identify_data = ""
        self._init_messages(DefaultMessages)
    def check_packet(self, s):
        if len(s) < MESSAGE_MIN:
            return 0
        msglen = ord(s[MESSAGE_POS_LEN])
        if msglen < MESSAGE_MIN or msglen > MESSAGE_MAX:
            return -1
        msgseq = ord(s[MESSAGE_POS_SEQ])
        if (msgseq & ~MESSAGE_SEQ_MASK) != MESSAGE_DEST:
            return -1
        if len(s) < msglen:
            # Need more data
            return 0
        if s[msglen-MESSAGE_TRAILER_SYNC] != MESSAGE_SYNC:
            return -1
        msgcrc = s[msglen-MESSAGE_TRAILER_CRC:msglen-MESSAGE_TRAILER_CRC+2]
        crc = crc16_ccitt(s[:msglen-MESSAGE_TRAILER_SIZE])
        if crc != msgcrc:
            #logging.debug("got crc %s vs %s", repr(crc), repr(msgcrc))
            return -1
        return msglen
    def dump(self, s):
        msgseq = s[MESSAGE_POS_SEQ]
        out = ["seq: %02x" % (msgseq,)]
        pos = MESSAGE_HEADER_SIZE
        while 1:
            msgid = s[pos]
            mid = self.messages_by_id.get(msgid, self.unknown)
            params, pos = mid.parse(s, pos)
            out.append(mid.format_params(params))
            if pos >= len(s)-MESSAGE_TRAILER_SIZE:
                break
        return out
    def format_params(self, params):
        name = params.get('#name')
        mid = self.messages_by_name.get(name)
        if mid is not None:
            return mid.format_params(params)
        msg = params.get('#msg')
        if msg is not None:
            return "%s %s" % (name, msg)
        return str(params)
    def parse(self, s):
        msgid = s[MESSAGE_HEADER_SIZE]
        mid = self.messages_by_id.get(msgid, self.unknown)
        params, pos = mid.parse(s, MESSAGE_HEADER_SIZE)
        if pos != len(s)-MESSAGE_TRAILER_SIZE:
            print " ------> ERR ! MID: %s POS %i LEN %i"%(mid.name, pos, len(s)) #nutiu
            raise error("Extra data at end of message")
        #nutiu
        params['#name'] = mid.name
        print "      XXPARSE: " + mid.name
        return params
    def encode(self, seq, cmd):
        msglen = MESSAGE_MIN + len(cmd)
        seq = (seq & MESSAGE_SEQ_MASK) | MESSAGE_DEST
        out = [chr(msglen), chr(seq), cmd]
        out.append(crc16_ccitt(''.join(out)))
        out.append(MESSAGE_SYNC)
        return ''.join(out)
    def _parse_buffer(self, value):
        if not value:
            return []
        tval = int(value, 16)
        out = []
        for i in range(len(value) // 2):
            out.append(tval & 0xff)
            tval >>= 8
        out.reverse()
        return out
    def lookup_command(self, msgformat):
        parts = msgformat.strip().split()
        msgname = parts[0]
        mp = self.messages_by_name.get(msgname)
        if mp is None:
            raise error("Unknown command: %s" % (msgname,))
        if msgformat != mp.msgformat:
            raise error("Command format mismatch: %s vs %s" % (
                msgformat, mp.msgformat))
        return mp
    def create_command(self, msg):
        parts = msg.strip().split()
        if not parts:
            return ""
        msgname = parts[0]
        mp = self.messages_by_name.get(msgname)
        if mp is None:
            raise error("Unknown command: %s" % (msgname,))
        try:
            argparts = dict(arg.split('=', 1) for arg in parts[1:])
            for name, value in argparts.items():
                t = mp.name_to_type[name]
                if t.is_dynamic_string:
                    tval = self._parse_buffer(value)
                elif t.is_int:
                    tval = int(value, 0)
                else:
                    tval = value
                argparts[name] = tval
        except error as e:
            raise
        except:
            #logging.exception("Unable to extract params")
            raise error("Unable to extract params from: %s" % (msgname,))
        try:
            cmd = mp.encode_by_name(**argparts)
        except error as e:
            raise
        except:
            #logging.exception("Unable to encode")
            raise error("Unable to encode: %s" % (msgname,))
        return cmd
    def _fill_enumerations(self, enumerations):
        for add_name, add_enums in enumerations.items():
            enums = self.enumerations.setdefault(add_name, {})
            for enum, value in add_enums.items():
                if type(value) == type(0):
                    # Simple enumeration
                    enums[str(enum)] = value
                    continue
                # Enumeration range
                enum = enum_root = str(enum)
                while enum_root and enum_root[-1].isdigit():
                    enum_root = enum_root[:-1]
                start_enum = 0
                if len(enum_root) != len(enum):
                    start_enum = int(enum[len(enum_root):])
                start_value, count = value
                for i in range(count):
                    enums[enum_root + str(start_enum + i)] = start_value + i
    def _init_messages(self, messages, output_ids=[]):
        for msgformat, msgid in messages.items():
            msgid = int(msgid)
            print "%s %s"%(msgformat, msgid)
            if msgid in output_ids:
                self.messages_by_id[msgid] = OutputFormat(msgid, msgformat)
                continue
            msg = MessageFormat(msgid, msgformat, self.enumerations)
            self.messages_by_id[msgid] = msg
            self.messages_by_name[msg.name] = msg
    def process_identify(self, data, decompress=True):
        #nutiu
        print "------process_identify------"
        try:
            if decompress:
                data = zlib.decompress(data)
            self.raw_identify_data = data
            data = json.loads(data)
            self._fill_enumerations(data.get('enumerations', {}))
            commands = data.get('commands')
            responses = data.get('responses')
            output = data.get('output', {})
            all_messages = dict(commands)
            all_messages.update(responses)
            all_messages.update(output)
            self.command_ids = sorted(commands.values())
            self._init_messages(all_messages, output.values())
            self.config.update(data.get('config', {}))
            self.version = data.get('version', '')
            self.build_versions = data.get('build_versions', '')
        except error as e:
            raise
        except Exception as e:
            logging.exception("process_identify error")
            raise error("Error during identify: %s" % (str(e),))
    def get_enumerations(self):
        return dict(self.enumerations)
    def get_constants(self):
        return dict(self.config)
    class sentinel: pass
    def get_constant(self, name, default=sentinel, parser=str):
        if name not in self.config:
            if default is not self.sentinel:
                return default
            raise error("Firmware constant '%s' not found" % (name,))
        try:
            value = parser(self.config[name])
        except:
            raise error("Unable to parse firmware constant %s: %s" % (
                name, self.config[name]))
        return value
    def get_constant_float(self, name, default=sentinel):
        return self.get_constant(name, default, parser=float)
    def get_constant_int(self, name, default=sentinel):
        return self.get_constant(name, default, parser=int)
