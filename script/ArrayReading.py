#!/usr/bin/env python

import socket, sys, os, struct, array
from time import *
from fcntl import ioctl
import select
import threading
from can2RNET import *
import binascii
from MessageArray import change_mode

import json
from decimal import Decimal

import pickle

event_messages = {
 '181c0000': 'song',
 '0c180201': '0c180201 seen after change mode to angle 1',
 '00000062': 'chairAngle motor: 40 -running, 00 -stopped',
 '00000060': '00000060 seen after change mode to angle 3',
 '00000061': '00000061 seen after change mode to angle 4',
 '0c180200': '0c180200 seen after change mode to angle 5',
 '0c180101': '0c180101 seen after change mode to drive 6',
 '0c180100': '0c180100 seen after change mode to drive 7',
 '0c000003': 'Blinking button(on the left of panel)',
 '181c0200': 'song',
 '1c240001': 'Device is ready. UI is active.',
 '0c000200': 'unknown',
 '0c000004': 'Lightning Button',
 '0c040000': 'horn on',
 '0c040001': 'horn off',
 '0c000002': 'right turn button',
 '0c000001': 'left turn button',
 '00000051': 'seen when speed changing',
 '00000050': 'seen when speed changing',
 '14300101': 'PMtx angle/height motor current',
 '00000000': 'seen at power off',
 '00000002': 'seen at power off',
 '0000000c': 'used by JSM to check for canbus connection',
 '00000004': 'seen at power off',
 '0c000105': 'motor has stopped',
 '0c000106': 'motor is decelerating'
}

bus = opencansocket(0)

def sendarr(s, arr):
    for msg in arr:
        sleep(0.005)
        s.send(msg)
        print(str(dec2hex(msg.arbitration_id, 8)) + "#" + binascii.hexlify(msg.data))

def readMsgs(bus,array):
    flag = True
    global event_messages
    time1 = time() + 5
    while flag:
        msg = bus.recv()
        if event_messages.get(str(dec2hex(msg.arbitration_id, 8))) != None:
            if len(arrayk) < 21:
                array.append(msg)
        if time1 < time():
            flag = False
        else:
            print (time1 - time())

array1 = []
print("press any key to start reading first arr")
b = raw_input()
print ("starting reading messages in to first array")
readMsgs(bus,array1)

print("press any key to send arr")
b = raw_input()
sendarr(bus,array1)

print("press any key to start reading second arr")
b = raw_input()
array2 = []
readMsgs(bus, array2)

print("press any key to send arr1")
b = raw_input()
sendarr(bus,array2)

while True:
    a = str(raw_input())
    if a == "1":
        sendarr(bus,array1)
    elif a == "2":
        sendarr(bus,array2)
