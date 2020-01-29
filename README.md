# wheelchair
More detailed about this project: https://telegra.ph/Learning-to-Control-Electric-Wheelchair-with-ROS-via-Standard-CAN-bus-07-29
Source project is a https://github.com/redragonx/can2RNET
This package contains 3 nodes:
  pi_node and master_node to control wheelchair via canbus
  and reader_node to read all messages from canbus
CanBus:
  You should send messages in to the CanBus in special format FrameID#Data
  Frame ID is a dec number
  Data is byte array
 
 Nodes:
 1. Master_node:
  This node can read data from Joystick and publish messages to topic1, which has 3 fields x, y, and event.
  All of them is a string format-fields.
  Coordinates of Joystick should be send in hex format, where 9d(min) is a -99, and 64(max) is 100.
  Format of events may be in any format you want, you just need to define what should do a subscriber, when it will receive this.
 
  When node receive information from Joystick.
  Information about event contains time, number of button and it state
  jnumber is a number of button
  jvalue is a state of button. val = 1 if pressed, val = 0 if unpressed
  Each button has own number.
 
  If you want to add functions for buttons you need to detect what number it has and create two clauses:
    when this button pressed and unpressed.
    
 2. Pi_node
  This node is a subscriber to topic1, reads data from it and sends to Can it.
  Also it makes another important function. It turns off Joystick on wheelchair using special function RNET_JSMerror_exploit
  This function should wait for Can message in bus, which is from Joystick on wheelchair. This message in format 02000M00#XxYy
  Where M is a device number. In my case M = 0.
 
  Currently waiting function does not work correct, so there is default JoyId 02000000 in programm, in case of timeout of waiting frame
 
  When JoyId is known by node, we need to turn off Joystick panel on wheelchair.
  It is necessary because, when we will send coordinates of our Joy it will send its own coordinates, which will cause delays
  and errors while controlling wheelchair
 
  We can induce error by sending special message for 3 times "0c000000#". It realized in function induce_JSM_error.
 
  Another to important parts of node is function of reading messages from topic, which updates global variables of X, Y and events,
  and thread of sending messages of X and Y to Bus.
 
  In callback function of reading messages from topic you can define what should do when received some new event.
 
  3. Reader_node:
    This node contain three dictionaries:
      periodic frames dictionary, it contain description of messages, which are send every time, when wheelchair is active.
      For example its serial number or battery level.
      
   event frames dictionary, it contain description of messages, which are send when something on Control panel pressed,
      or answers from another parts of wheelchair, like messages of that motors are stopping or stopped.
      
   drive event dictionary contains message, which is send by Joystick
      
   For each of those dictionaries node has topic and it publish there what exact message it received from bus with data
     and its description
     
   Not every message is known by dictionary, so which is not know will send to event messages topic.
     Also not all messages in dictionary have full description and it is written only when it was seen, for example.
     
      
Important notice about ROS:
 if you want to use nodes on two different machines you need to follow this guide:
  https://razbotics.wordpress.com/2018/01/23/ros-on-multiple-computers-connecting-raspberry-pi-with-pc-over-lan/

     
     
     

     
     
     
     
     
     
     
