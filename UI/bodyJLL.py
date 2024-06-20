#!/usr/bin/env python3
''' JLL  2023.3.6, 4.13
for 230412
from /openpilot/selfdrive/ui/tests/body.py

(sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
(sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python bodyJLL.py
'''
import time
import cereal.messaging as messaging

if __name__ == "__main__":
  while True:
    pm = messaging.PubMaster(['carParams', 'carState'])
    while True:
      msg = messaging.new_message('carParams')
      msg.carParams.carName = "COMMA BODY"
      msg.carParams.notCar = True  # > home.cc > "Body Face" > mouseDoubleClickEvent
      msg.carParams.openpilotLongitudinalControl = True  # from uiview.py
      pm.send('carParams', msg)

      for b in range(100, 0, -1):
        msg = messaging.new_message('carState')
        msg.carState.charging = True  # > body.cc > getCharging()
        msg.carState.fuelGauge = b / 100.  # > body.cc > getFuelGauge()
        pm.send('carState', msg)
        time.sleep(0.1)

      time.sleep(1)
