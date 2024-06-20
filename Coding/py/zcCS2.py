CAN   JLL, 2021.4.29-5.6
Copy and Paste Code Snippets to zc.py
--------------------------------------------------
CCCCCCCCCCCCCCCCCCCCCCCC Code Snippets
'''
CAN 9    JLL, 2021.4.20, 4.27, 5.6
jinn@Liu:~/can$ python zc.py
(OP) jinn@Liu:~/openpilot/opendbc/can$ python dbc.py

https://medium.com/@energee/add-support-for-your-car-to-comma-ai-openpilot-3d2da8c12647
'''
a = 0xff00000000000000 >> 56
b = 0xff00000000000000 >> 40
print(' 0xff00000000000000 >> 56 = ', a)
print(' 0xff00000000000000 >> 40 = ', b)
print(' bin(0xff00000000000000) = ', bin(0xff00000000000000))
print(' bin(a)                  = ', bin(a))
print(' bin(b)                  = ', bin(b))
print("$$$$$  0x80 = ", 0x80)
print(' bin(0x80) = ', bin(0x80))
print(' bin(0x800) = ', bin(0x800))

CCCCCCCCCCCCCCCCCCCCCCCC
'''
CAN 8    Do CAN 3 first
jinn@Liu:~/can$ python zc.py
Input: /home/jinn/can/motohawk.dbc
'''
import os
from binascii import hexlify
import cantools

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
MOTOHAWK_PATH = os.path.join(SCRIPT_DIR,'motohawk.dbc')
database = cantools.db.load_file(MOTOHAWK_PATH)

message = {
    'Temperature': 250.1,
    'AverageRadius': 3.2,
    'Enable': 'Enabled'
}
encoded = database.encode_message('ExampleMessage', message)
decoded = database.decode_message('ExampleMessage', encoded)

print('Message:', message)
print('$$$$$ encoded = ', encoded)
print('Encoded:', hexlify(encoded).decode('ascii'))
print('Decoded:', decoded)

CCCCCCCCCCCCCCCCCCCCCCCC
'''
CAN 7    Do CAN 3 first
--- Terminal 1
jinn@Liu:~/can$ candump vcan0 | python3 -m cantools decode motohawk.dbc
--- Terminal 2
jinn@Liu:~/can$ python zc.py
Input: /home/jinn/can/motohawk.dbc
'''
import can
import time
import cantools
from pprint import pprint

# from https://github.com/eerimoq/cantools
db = cantools.database.load_file('motohawk.dbc')
print("$$$$$  db = ", db.messages)
example_message = db.get_message_by_name('ExampleMessage')
pprint(example_message.signals)

can_bus = can.interface.Bus(channel='vcan0', bustype='socketcan')
data = example_message.encode({'Temperature': 255.1, 'AverageRadius': 3.2, 'Enable': 1})
message = can.Message(arbitration_id=example_message.frame_id, data=data)
can_bus.send(message)
#message = can_bus.recv()
#db.decode_message(message.arbitration_id, message.data)
#{'AverageRadius': 3.2, 'Enable': 'Enabled', 'Temperature': 250.09}

CCCCCCCCCCCCCCCCCCCCCCCC
'''
CAN 6    Do CAN 3 first
--- Terminal 1
jinn@Liu:~/can$ python zc.py listen vcan0
--- Terminal 2
jinn@Liu:~/can$ python zc.py send vcan0 1ff deadbeef
'''
# Copyright (c) 2016 Alex Bencz
# http://www.bencz.com/hacks/2016/07/10/python-and-socketcan/
import sys
import socket
import argparse
import struct
import errno

class CANSocket(object):
  FORMAT = "<IB3x8s"
  FD_FORMAT = "<IB3x64s"
  CAN_RAW_FD_FRAMES = 5

  def __init__(self, interface=None):
    self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    if interface is not None:
      self.bind(interface)

  def bind(self, interface):
    self.sock.bind((interface,))
    self.sock.setsockopt(socket.SOL_CAN_RAW, self.CAN_RAW_FD_FRAMES, 1)

  def send(self, cob_id, data, flags=0):
    cob_id = cob_id | flags
    can_pkt = struct.pack(self.FORMAT, cob_id, len(data), data)
    self.sock.send(can_pkt)

  def recv(self, flags=0):
    can_pkt = self.sock.recv(72)
    if len(can_pkt) == 16:
      cob_id, length, data = struct.unpack(self.FORMAT, can_pkt)
    else:
      cob_id, length, data = s--- Using Python3 With Socketcan  http://www.bencz.com/hacks/2016/07/10/python-and-socketcan/
truct.unpack(self.FD_FORMAT, can_pkt)
    cob_id &= socket.CAN_EFF_MASK
    return (cob_id, data[:length])

def format_data(data):
    return ''.join([hex(byte)[2:] for byte in data])

def generate_bytes(hex_string):
    if len(hex_string) % 2 != 0:
      hex_string = "0" + hex_string
    int_array = []
    for i in range(0, len(hex_string), 2):
        int_array.append(int(hex_string[i:i+2], 16))
    return bytes(int_array)

def send_cmd(args):
    try:
      s = CANSocket(args.interface)
    except OSError as e:
      sys.stderr.write('Could not send on interface {0}\n'.format(args.interface))
      sys.exit(e.errno)
    try:
      cob_id = int(args.cob_id, 16)
    except ValueError:
      sys.stderr.write('Invalid cob-id {0}\n'.format(args.cob_id))
      sys.exit(errno.EINVAL)
    s.send(cob_id, generate_bytes(args.body), socket.CAN_EFF_FLAG if args.extended_id else 0)

def listen_cmd(args):
    try:
      s = CANSocket(args.interface)
    except OSError as e:
      sys.stderr.write('Could not listen on interface {0}\n'.format(args.interface))
      sys.exit(e.errno)
    print('Listening on {0}'.format(args.interface))
    while True:
        cob_id, data = s.recv()
        print('%s %03x#%s' % (args.interface, cob_id, format_data(data)))

def parse_args():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers()
    send_parser = subparsers.add_parser('send', help='send a CAN packet')
    send_parser.add_argument('interface', type=str, help='interface name (e.g. vcan0)')
    send_parser.add_argument('cob_id', type=str, help='hexadecimal COB-ID (e.g. 10a)')
    send_parser.add_argument('body', type=str, nargs='?', default='',
      help='hexadecimal msg body up to 8 bytes long (e.g. 00af0142fe)')
    send_parser.add_argument('-e', '--extended-id', action='store_true', default=False,
      help='use extended (29 bit) COB-ID')
    send_parser.set_defaults(func=send_cmd)
    listen_parser = subparsers.add_parser('listen', help='listen for and print CAN packets')
    listen_parser.add_argument('interface', type=str, help='interface name (e.g. vcan0)')
    listen_parser.set_defaults(func=listen_cmd)
    return parser.parse_args()

def main():
    args = parse_args()
    args.func(args)

if __name__ == '__main__':
    main()

CCCCCCCCCCCCCCCCCCCCCCCC
'''
CAN 5    Do CAN 3 first
--- Terminal 1
jinn@Liu:~/can$ candump vcan0
--- Terminal 2
jinn@Liu:~/can$ python zc.py
Input: /home/jinn/can/motohawk.dbc
Output: in Terminals

Read --- CAN DBC File Explained - A Simple Intro
     --- CANpy/docs/DBC_Specification.md
     --- An Introduction to J1939 and DBC files
     --- CAN BUS tools: https://cantools.readthedocs.io/en/latest/
     --- Collection of CAN bus packages and tools
'''
# https://github.com/eerimoq/cantools
import can
import time
import cantools
from pprint import pprint

db = cantools.database.load_file('motohawk.dbc')
print("$$$$$  db = ", db.messages)
example_message = db.get_message_by_name('ExampleMessage')
pprint(example_message.signals)
print("$$$$$  message ID 0x1f0 = ", 0x1f0)

can_bus = can.interface.Bus(channel='vcan0', bustype='socketcan')
data = example_message.encode({'Temperature': 250.1, 'AverageRadius': 3.2, 'Enable': 1})
message = can.Message(arbitration_id=example_message.frame_id, data=data)
can_bus.send(message)
print("$$$$$  message ID example_message.frame_id = ", example_message.frame_id)
print("$$$$$  message ID 0x7EE = ", 0x7EE)
print("$$$$$  message ID 0x000001F0 = ", 0x000001F0)

# https://python-can.readthedocs.io/en/2.1.0/interfaces/socketcan.html
def producer(id):
    # :param id: Spam the bus with messages including the data id."""
    bus = can.interface.Bus(channel='vcan0', bustype='socketcan')
    for i in range(id):
        print("$$$$$  Sending message # ", i)
        msg = can.Message(arbitration_id=0xc0ffee, data=[id, i, 0, 1, 3, 1, 4, 1], extended_id=False)
        bus.send(msg)
    time.sleep(1)
producer(3)
print("$$$$$  message ID 0xc0ffee = ", 0xc0ffee)

CCCCCCCCCCCCCCCCCCCCCCCC
'''
CAN 4    Do CAN 3 first
https://github.com/eerimoq/cantools
'''
# https://stackoverflow.com/questions/64081604/how-to-run-a-simulation-of-can-messages-on-python
import time
import can
bus = can.interface.Bus(interface='virtual', bustype='socketcan',
                        channel='vcan0', bitrate=500000)
def producer(bus, id):
    for i in range(3):
        msg = can.Message(arbitration_id=0xc0ffee,
                          data=[id, i, 0, 1, 3, 1, 4, 1], is_extended_id=False)
        try:
            bus.send(msg)
            print("Message sent on {}".format(bus.channel_info))
            time.sleep(1)
        except can.CanError:
            print("Message NOT sent")
producer(bus, 1)
