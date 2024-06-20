#!/usr/bin/env python3
''' JLL  2023.3.5, 4.13
for 230305, 230412, 230413
from /openpilot/selfdrive/debug/uiview.py
'''
import time

from cereal import log, messaging
from selfdrive.manager.process_config import managed_processes

if __name__ == "__main__":
    #procs = ['camerad', 'ui', 'modeld', 'calibrationd']  # camerad is not meant to run on PC
  procs = ['ui']
  for p in procs:
    managed_processes[p].start()

  pm = messaging.PubMaster(['deviceState', 'pandaStates'])  # 230412
    #pm = messaging.PubMaster(['controlsState', 'deviceState', 'pandaStates'])
  msgs = {s: messaging.new_message(s) for s in ['deviceState']}

  msgs['deviceState'].deviceState.started = True
  msgs['pandaStates'] = messaging.new_message('pandaStates', 1)
  msgs['pandaStates'].pandaStates[0].ignitionLine = True  # 0 mph
  msgs['pandaStates'].pandaStates[0].pandaType = log.PandaState.PandaType.uno  # VEHICLE ONLINE or NO PANDA

  try:
    while True:
      time.sleep(1 / 100)  # continually send, rate doesn't matter
      for s in msgs:
        pm.send(s, msgs[s])  # this in while loop gives UserWarning
  except KeyboardInterrupt:  # Python KeyboardInterrupt
    for p in procs:
      managed_processes[p].stop()
