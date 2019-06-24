import ctypes
import enum

from std_struct import StandardStruct
from util import hex_string


BROADCAST_ADDRESS   = 0xFFFF
BASE_ADDRESS        = 1
APP_MESSAGE_SIZE    = 127


class Endpoint(enum.IntEnum):
    ECHO        = 0x00
    JOIN        = 0x01
    BEACON      = 0x02
    BASE_REPORT = 0x03
    RANGE       = 0x04
    READ_CONF   = 0x05
    WRITE_CONF  = 0x06
    RESET       = 0x07


class NodeType(enum.IntEnum):
    NONE        = 0x00
    BRIDGE      = 0x01
    BASE        = 0x02
    ANCHOR      = 0x03
    TAG         = 0x04


class MacHeader(StandardStruct):

    _fields_ = [
        ("frame_control",   ctypes.c_uint16),
        ("number",          ctypes.c_uint8),
        ("pan_id",          ctypes.c_uint16),
        ("dst",             ctypes.c_uint16),
        ("src",             ctypes.c_uint16),
    ]


class AppHeader(StandardStruct):

    _fields_ = [
        ("len",         ctypes.c_uint16),
        ("endpoint",    ctypes.c_uint8),
        ("step",        ctypes.c_uint8),
    ]


class MacFrame(StandardStruct):

    _string_include_    = ['payload_hex']
    _string_exclude_    = ['payload']

    _fields_ = [
        ('mac_header',  MacHeader),
        ('app_header',  AppHeader),
        ('payload', ctypes.c_uint8 * (APP_MESSAGE_SIZE + 2)),
    ]

    @property
    def payload_len(self):
        return self.app_header.len - ctypes.sizeof(AppHeader)

    @property
    def payload_truncated(self):
        return bytearray(self.payload[:self.payload_len])

    @property
    def payload_hex(self):
        return hex_string(self.payload_truncated)

    def to_one_liner(self):
        return ('%04X to %04X, %s step %d: %s' % (
                self.mac_header.src,
                self.mac_header.dst,
                Endpoint(self.app_header.endpoint).name,
                self.app_header.step,
                self.payload_hex))

    def serialize(self):
        length = ctypes.sizeof(MacHeader) + self.app_header.len
        return bytearray(self)[:length]


class JoinRequest(StandardStruct):

    _fields_ = [
        ("node_type",   ctypes.c_uint16),
        ("uuid",        ctypes.c_uint8 * 16),
    ]


class JoinResponse(StandardStruct):

    _fields_ = [
        ("address",     ctypes.c_uint16),
        ("uuid",        ctypes.c_uint8 * 16),
    ]


class BaseReportItem(StandardStruct):

    _fields_ = [
        ("tag",         ctypes.c_uint16),
        ("anchors",     ctypes.c_uint16 * 3),
    ]


# Support only 4 tags for now
class BaseReport(StandardStruct):

    _fields_ = [
        ("items",       BaseReportItem*4),
    ]


# Ranging report!
class RangeReport(StandardStruct):

    _fields_ = [
        ("tag_address",     ctypes.c_uint16),
        ("anchor_address",  ctypes.c_uint16),
        ("poll_order_idx",  ctypes.c_uint8),
        ("poll_send",       ctypes.c_uint32),
        ("poll_rcv",        ctypes.c_uint32),
        ("rsp_send",        ctypes.c_uint32),
        ("rsp_rcv",         ctypes.c_uint32),
        ("final_send",      ctypes.c_uint32),
        ("final_rcv",       ctypes.c_uint32),
    ]

