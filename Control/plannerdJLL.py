#!/usr/bin/env python3
''' JLL  2023.8.14
for 230814
from /openpilot/selfdrive/controls/plannerd.py

(sconsvenv) jinn@Liu:~/openpilot$ python plannerdJLL.py
'''
from cereal import car
from common.params import Params
#from selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from selfdrive.controls.lib.lateral_plannerJLL import LateralPlanner
  #from selfdrive.controls.lib.lateral_planner import LateralPlanner
import cereal.messaging as messaging

def plannerd_thread(sm=None, pm=None):
  params = Params()
  CP = car.CarParams.from_bytes(params.get("CarParams", block=True))
  print("#--- CP.carName =", CP.carName)

  #longitudinal_planner = LongitudinalPlanner(CP)
  lateral_planner = LateralPlanner(CP)

  if sm is None:
    sm = messaging.SubMaster(['carControl', 'carState', 'controlsState', 'radarState', 'modelV2'],
                             poll=['radarState', 'modelV2'], ignore_avg_freq=['radarState'])

  if pm is None:
      #pm = messaging.PubMaster(['longitudinalPlan', 'lateralPlan', 'uiPlan'])
    pm = messaging.PubMaster(['lateralPlan'])

  #while True:
  sm.update()

  if sm.updated['modelV2']:
    lateral_planner.update(sm)
    #lateral_planner.publish(sm, pm)
    #longitudinal_planner.update(sm)
    #longitudinal_planner.publish(sm, pm)

def main(sm=None, pm=None):
  plannerd_thread(sm, pm)


if __name__ == "__main__":
  main()
