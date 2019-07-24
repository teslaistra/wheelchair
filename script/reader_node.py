#!/usr/bin/env python

import socket, sys, os, struct, array
from time import *
from fcntl import ioctl
import select
import threading
from can2RNET import *
import rospy
from std_msgs.msg import String
import binascii
periodic_messages = {
'1c0c0100': 'JSMrx battery power level in % Xx = 0x00 - 0x64 -p',
 '14300100': 'PMtx drive motor current -p',
 '03c30f0f': 'JSMtx device heartbeat -p',
 '0c140200': 'smth system JSM 1',
 '1c300104': 'PMtx distance counter',
 '0000000e': 'serial number',
 '0c140100': 'smth system JSM 2',
}
drive_control = {
    '02000000': 'JSM frame - drive control '
}
event_messages = {

 '181c0000': 'song',
 '0c180201': 'seen after change mode to angle',
 '00000062': 'seen after change mode to angle',
 '00000060': 'seen after change mode to angle',
 '00000061': 'seen after change mode to angle',
 '0c180200': 'seen after change mode to angle',
 '0c180101': 'seen after change mode to drive',
 '0c180100': 'seen after change mode to drive',
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
 '00000004': 'seen at power off'

}
def dec2hex(dec, hexlen):  # convert dec to hex with leading 0s and no '0x'
    h = hex(int(dec))[2:]
    l = len(h)
    if h[l - 1] == "L":
        l -= 1  # strip the 'L' that python int sticks on
    if h[l - 2] == "x":
        h = '0' + hex(int(dec))[1:]
    return ('0' * hexlen + h)[l:l + hexlen]

if __name__ == "__main__":
        node = rospy.init_node('PiNode')
        DrivePublisher = rospy.Publisher("JoyXY", String, queue_size=0)
        EventPublisher = rospy.Publisher("JoyEvents", String, queue_size=0)
        PeriodicPublisher = rospy.Publisher("PeriodicsMsgs", String, queue_size=0)

        # topic
        global cansocket
        bus = opencansocket(0)
        while not rospy.is_shutdown():
            msg = bus.recv()
            if drive_control.get(str(dec2hex(msg.arbitration_id, 8))) != None:
                data = binascii.hexlify(msg.data)
                Xdata = int(data[0:2],16)
                Ydata = int(data[2:4],16)
                text = "Recieved cordinates of Joy: x = "  + str(Xdata) + " y = " + str(Ydata)
                msg = String()
                msg.data = text
                DrivePublisher.publish(msg)
            elif event_messages.get(str(dec2hex(msg.arbitration_id, 8))) != None:
                data = binascii.hexlify(msg.data)
                text = "Recieved message about/from: " + event_messages.get(str(dec2hex(msg.arbitration_id, 8))) + " with data " + binascii.hexlify(msg.data)
                msg= String()
                msg.data = text
                EventPublisher.publish(msg)
            elif periodic_messages.get(str(dec2hex(msg.arbitration_id, 8))) != None:
                data = binascii.hexlify(msg.data)
                text = "Recieved periodic message about/from: " + periodic_messages.get(str(dec2hex(msg.arbitration_id, 8))) + " with data " + binascii.hexlify(msg.data)
                msg = String()
                msg.data = text
                PeriodicPublisher.publish(msg)
            else:
                text = "Unknown Message with ID: " + str(dec2hex(msg.arbitration_id, 8)) + " data: "  + binascii.hexlify(msg.data)
                msg = String()
                msg.data = text
                EventPublisher.publish(msg)


