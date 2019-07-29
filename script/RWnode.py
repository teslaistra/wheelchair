#!/usr/bin/env python
# joystick based on: https://www.kernel.org/doc/Documentation/input/joystick-api.txt

import socket, sys, os, struct, array
from time import *
from fcntl import ioctl
import select
import threading
from can2RNET import *
import rospy
from wheelchair.msg import joy
from std_msgs.msg import String
from reader_node import periodic_messages, drive_control, event_messages
from wheelchair.msg import canMSG
from wheelchair.msg import joy


def dec2hex(dec, hexlen):  # convert dec to hex with leading 0s and no '0x'
    h = hex(int(dec))[2:]
    l = len(h)
    if h[l - 1] == "L":
        l -= 1  # strip the 'L' that python int sticks on
    if h[l - 2] == "x":
        h = '0' + hex(int(dec))[1:]
    return ('0' * hexlen + h)[l:l + hexlen]


def send_joystick_canframe(s, joy_id):
    # packing data from joystick and sending it
    mintime = .01
    nexttime = time() + mintime
    priorjoyx = joyx
    priorjoyy = joyy
    while rnet_threads_running:
        joyframe = joy_id + '#' + dec2hex(joyx, 2) + dec2hex(joyy, 2)
        cansend(s, joyframe)
        nexttime += mintime
        t = time()
        if t < nexttime:
            sleep(nexttime - t)
        else:
            nexttime += mintime


def wait_joystickframe(cansocket, t):  # won't work corrcetly, but will return correct frame id
    frameid = ''
    while frameid[0:3] != '020':
        # just look for joystick frame ID (no extended frame)
        msg = cansocket.recv()
        frameid = str(dec2hex(msg.arbitration_id, 8))
        print(frameid)
        if t > time():
            print("JoyFrame wait timed out ")
            return ('02000000')
    return (frameid)


def induce_JSM_error(cansocket):
    for i in range(0, 3):
        cansend(cansocket, '0c000000#')


def RNET_JSMerror_exploit(cansocket):
    t = time() + 0.20
    print("Waiting for joy frame")
    joy_id = wait_joystickframe(cansocket, t)
    print("Using joy frame: " + joy_id)
    induce_JSM_error(cansocket)
    print("3 x 0c000000# sent")
    return (joy_id)


def RNETsetSpeedRange(cansocket, speed_range):
    if speed_range >= 0 and speed_range <= 0x64:
        cansend(cansocket, '0a040100#' + dec2hex(speed_range, 2))
    else:
        print('Invalid RNET SpeedRange: ' + str(speed_range))


def RNETshortBeep(cansocket):
    cansend(cansocket, "181c0100#0260000000000000")


def RNETplaysong(cansocket):
    cansend(cansocket, "181C0100#2056080010560858")
    sleep(.77)
    cansend(cansocket, "181C0100#105a205b00000000")


def watch_and_wait():
    time1 = time()

    while threading.active_count() > 0 and not rospy.is_shutdown():

        msg = bus.recv()
        if drive_control.get(str(dec2hex(msg.arbitration_id, 8))) != None:
            data = binascii.hexlify(msg.data)
            Xdata = data[0:2]
            Ydata = data[2:4]
            text = "Recieved cordinates of Joy: x = " + str(Xdata) + " y = " + str(Ydata)
            msg = String()
            msg.data = text
            DrivePublisher.publish(msg)
        elif event_messages.get(str(dec2hex(msg.arbitration_id, 8))) != None:
            data = binascii.hexlify(msg.data)


            text = "Recieved message about/from: " + event_messages.get(str(dec2hex(msg.arbitration_id, 8))) + " with data " + binascii.hexlify(msg.data)
            msg = String()
            msg.data = text
            EventPublisher.publish(msg)
        elif periodic_messages.get(str(dec2hex(msg.arbitration_id, 8))) != None:
            data = binascii.hexlify(msg.data)
            text = "Recieved periodic message about/from: " + periodic_messages.get(
                str(dec2hex(msg.arbitration_id, 8))) + " with data " + binascii.hexlify(msg.data)
            msg = String()
            msg.data = text
            PeriodicPublisher.publish(msg)
        else:
            text = "Unknown Message with ID: " + str(dec2hex(msg.arbitration_id, 8)) + " data: " + binascii.hexlify(
                msg.data)
            msg = String()
            msg.data = text
            EventPublisher.publish(msg)
        if time()>time1:
            print('X: ' + dec2hex(joyx, 2) + '\tY: ' + dec2hex(joyy, 2))
            time1 = time() + 0.5



def kill_rnet_threads():
    global rnet_threads_running
    rnet_threads_running = False


def callback_for_msg(msg):
    global joyx
    global joyy
    global cansocket
    joyx = int(msg.x, 16) & 0xFF
    joyy = int(msg.y, 16) & 0xFF
    priorspeedrange = 0

    if msg.event[0].decode() == 's':
        speed_range = int(msg.event[2:], 16)
        if speed_range != priorspeedrange:
            print("received SpeedRange: " + str(speed_range))
            RNETsetSpeedRange(cansocket, speed_range)
            RNETshortBeep(cansocket)
            priorspeedrange = speed_range
    elif msg.event[0] == 'b':
        if msg.event[2:4] == 'h0':
            cansend(cansocket, "0C040101#")  # horn off
        if msg.event[2:4] == 'h1':
            RNETplaysong(cansocket)
            cansend(cansocket, "0C040100#")  # horn on
        if msg.event[2:4] == 'fl':
            cansend(cansocket, "0C000404#")
        if msg.event[2:4] == 'tl':
            cansend(cansocket, "0C000401#")
        if msg.event[2:4] == 'tr':
            cansend(cansocket, "0C000402#")


if __name__ == "__main__":

    node = rospy.init_node('PiNode') #node to control wheelchair
    subscriber = rospy.Subscriber("/topic1", joy, callback_for_msg)

    node = rospy.init_node('ReaderNode') #node to publish messages from CAN bus
    DrivePublisher = rospy.Publisher("JoyXY", String, queue_size=0)
    EventPublisher = rospy.Publisher("JoyEvents", String, queue_size=0)
    PeriodicPublisher = rospy.Publisher("PeriodicsMsgs", String, queue_size=0)

    global rnet_threads_running
    rnet_threads_running = True
    global cansocket
    cansocket = opencansocket(0)
    if cansocket != '':
        print(cansocket)
        # init /dev joystick
        global joyx
        global joyy
        joyx = 0
        joyy = 0

        joy_id = RNET_JSMerror_exploit(cansocket)
        playsongthread = threading.Thread(target=RNETplaysong, args=(cansocket,))

        speed_range = 00
        RNETsetSpeedRange(cansocket, speed_range)
        induce_JSM_error(cansocket)

        sendjoyframethread = threading.Thread(target=send_joystick_canframe, args=(cansocket, joy_id,))
        sendjoyframethread.start()
        playsongthread.start()

        watch_and_wait()
        cansocket.shutdown()
    kill_rnet_threads()
    print("Exiting")

