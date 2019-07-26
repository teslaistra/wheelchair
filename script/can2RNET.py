#!/Python2.7

#Python2 canbus functions

import socket
import struct
import sys
from time import *
import binascii
import threading
import can
import array
#all functions take CAN messages as a string in "CANSEND" (from can-utils) format
"""
FORMAT FOR CANSEND (matches candump -l)
    <can_id>#{R|data}          for CAN 2.0 frames
    <can_id>##<flags>{data}    for CAN FD frames

<can_id> can have 3 (SFF) or 8 (EFF) hex chars
{data} has 0..8 (0..64 CAN FD) ASCII hex-values (optionally separated by '.')
<flags> a single ASCII Hex value (0 .. F) which defines canfd_frame.flags

e.g. 5A1#11.2233.44556677.88 / 123#DEADBEEF / 5AA# / 123##1 / 213##311
     1F334455#1122334455667788 / 123#R for remote transmission request.
"""

def build_frame(canstr):
    if not '#' in canstr:
        print('build_frame: missing #')
        return 'Err!'
    cansplit = canstr.split('#')
    len_datstr = len(cansplit[1])
    if len_datstr <= 16 and not len_datstr & 1:
        candat = binascii.unhexlify(cansplit[1])
    elif not len_datstr:
        candat = ''
    else:
        print('build_frame: cansend data format error: ' + canstr)
        return 'Err!'
    return map(ord, candat)

def dissect_frame(msg):
    return str(dec2hex(msg.arbitration_id, 8)) + "#" + binascii.hexlify(msg.data)

def cansend(s,cansendtxt):

    cansplit = cansendtxt.split('#')
    out=build_frame(cansendtxt)

    if out != 'Err!':
        c1 = build_frame("#"+cansplit[1])
        c = bytearray(c1)
        msg = can.Message(arbitration_id=int(cansplit[0],16), data=c)
        s.send(msg)

def opencansocket(busnum):
    busnum='can'+str(busnum)
    bus = can.interface.Bus(channel = busnum, bustype='socketcan')
    print ("connected to CAN")
    return bus
