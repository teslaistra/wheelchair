#!/usr/bin/env python

import socket, sys, os, struct, array
from time import *
from fcntl import ioctl
import select
import threading
from can2RNET import *

import binascii
from reader_node import periodic_messages, drive_control, event_messages, dec2hex
from pi_node import induce_JSM_error

bus = opencansocket(0)

flag= True
time1 = time() + 10
a = []
print ("starting reading messages")
while flag:
    msg = bus.recv()
    if event_messages.get(str(dec2hex(msg.arbitration_id, 8))) != None:
        if a.lenght < 10:
            a.append(msg)
    if time1 < time():
        print (time1-time())
        flag = False

print("press any key to send arr")
b = input()

for msg in a:
    sleep(0.005)
    bus.send(msg)