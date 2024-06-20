#!/usr/bin/env python3
''' JLL  2023.3.6, 4.13
for 230413
from /openpilot/selfdrive/debug/cycle_alerts.py
'''
import time

from cereal import car, log
import cereal.messaging as messaging
from common.realtime import DT_CTRL
from selfdrive.car.honda.interface import CarInterface
from selfdrive.controls.lib.events import ET, Events
from selfdrive.controls.lib.alertmanager import AlertManager

EventName = car.CarEvent.EventName

def cycle_alerts(duration=200, is_metric=False):
  # debug alerts
  alerts = [
    (EventName.joystickDebug, ET.WARNING),  # = (34, 'warning') from (car.capnp, class ET)
  ]

  CS = car.CarState.new_message()
  CP = CarInterface.get_non_essential_params("HONDA CIVIC 2016")

  sm = messaging.SubMaster(['testJoystick'])
  pm = messaging.PubMaster(['controlsState', 'pandaStates', 'deviceState'])

  events = Events()
  AM = AlertManager()

  frame = 0
  while True:
    for alert, et in alerts:
      events.clear()
      events.add(alert)

      sm['testJoystick'].axes = [-0.3, 0.7]  # =  [gas, steer]

      a = events.create_alerts([et, ], [CP, CS, sm, is_metric, 0])
      AM.add_many(frame, a)
      alert = AM.process_alerts(frame, [])
      print(alert)
      for _ in range(duration):
        dat = messaging.new_message()
        dat.init('controlsState')
        dat.controlsState.enabled = False

        if alert:
          dat.controlsState.alertText1 = alert.alert_text_1
          dat.controlsState.alertText2 = alert.alert_text_2
          dat.controlsState.alertSize = alert.alert_size
          dat.controlsState.alertStatus = alert.alert_status
          dat.controlsState.alertBlinkingRate = alert.alert_rate
          dat.controlsState.alertType = alert.alert_type
          dat.controlsState.alertSound = alert.audible_alert
        pm.send('controlsState', dat)

        dat = messaging.new_message()
        dat.init('deviceState')
        dat.deviceState.started = True
        pm.send('deviceState', dat)

        dat = messaging.new_message('pandaStates', 1)
        dat.pandaStates[0].ignitionLine = True
        dat.pandaStates[0].pandaType = log.PandaState.PandaType.uno
        pm.send('pandaStates', dat)

        frame += 1
        time.sleep(DT_CTRL)

if __name__ == '__main__':
  cycle_alerts()
