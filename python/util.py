"""
Miscellaneous helper functions
"""
try:
    from Queue import Empty
except ImportError:
    from queue import Empty

import collections
import logging
import time
import sys


def clamp(num, lower=0, upper=None):
    """ Clamp a value so that it must be between a lower and upper value"""
    if upper is None:
        upper = float('inf')
    if num < lower:
        return lower
    if num > upper:
        return upper
    return num


def flush_queue(queue):
    """ Flush a standard Python queue """
    try:
        while not queue.empty():
            queue.get(False)
    except Empty:
        pass


def to_bytearray(thing):
    """
    Turns a thing into a bytearray. Behavior:
    - Short-circuit if thing is already a bytearray
    - Wraps all non-iterables in a list, e.g. 5 -> [5]
    - If thing is a unicode string, turn it into ASCII string
    - If the first element of the iterable is a number,
      convert all elements to bytes via chr()
    - Return bytearray(thing)
    All of this is convenient but expensive - so use in low-bandwidth
    applications, or provide a bytearray up front to short-circuit.
    """
    if isinstance(thing, bytearray):
        return thing
    if not thing:
        return ''
    if not isinstance(thing, collections.Iterable):
        thing = [thing]
    if isinstance(thing, unicode):
        thing = str(thing)
    if isinstance(thing[0], int) or isinstance(thing[0], float):
        thing = [chr(bite) for bite in thing]
    return bytearray(thing)


def print_now(msg):
    """ Immediate print to command line """
    sys.stdout.write(msg)
    sys.stdout.flush()


def rgb_from_string(color):
    """ Convert a color's name (string) to 0-255 RGB tuple """
    color_map = {
        'red'     :   (255,  0,   0),
        'green'   :   (0,    255, 0),
        'blue'    :   (0,    0,   255),
        'yellow'  :   (255,  255, 0),
        'teal'    :   (0,    255, 255),
        'purple'  :   (255,  0,   255),
        'blank'   :   (0,    0,   0),
        'white'   :   (255,  255, 255),
    }
    return color_map[color]


def read_bytes(q, size=1, timeout=None):
    """
    Read size bytes from a queue with timeout.
    Mimics serial.Serial.read() behavior.
    """
    ret = ''
    start = time.time()
    while len(ret) < size:
        try:
            ret += q.get(block=False)
        except Empty:
            pass
        if timeout and (time.time() - start > timeout):
            break
    return ret


def indent(string, leading_char='\t'):
    return leading_char + string.replace('\n', '\n\t')


def hex_string(val_list, bytes_per_line=16, fmt='%02X'):
    """
    Stringify one or a list of byte values to hex format
    Handles list, string, bytearray, array.array, etc.
    Set bytes_per_line=None to print as one line
    Format can be overridden, e.g. '0x%02X' if you want 0x prefix
    """
    if not hasattr(val_list, '__getitem__'):
        val_list = [val_list]
    if isinstance(val_list, bytearray):
        val_list = [int(val) for val in val_list]
    if isinstance(val_list, basestring):
        val_list = [ord(val) for val in val_list]
    if not bytes_per_line:
        bytes_per_line = len(val_list)
    group_i = [i for i in xrange(0, len(val_list), bytes_per_line)]
    groups  = [val_list[i:i+bytes_per_line] for i in group_i]
    lines   = [' '.join([fmt % val for val in group]) for group in groups]
    if not lines:
        return '[ nothing ]'
    return '\n'.join(lines)

def dev_log_config(level):
    """ More legible, thread-friendly log config than Python default """
    log_format = "%(levelname)-8s" + \
                 "%(threadName)-16s" + \
                 "%(filename)-24s" + \
                 "@%(lineno)-4s" + \
                 "\t\t%(message)s"
    logging.basicConfig(level=level, format=log_format)