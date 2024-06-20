#!/usr/bin/env python3
''' JLL  2023.8.15
for 230814
from /openpilot/aJLL/Control/plantLonJLL.py
(sconsvenv) jinn@Liu:~/openpilot/aJLL/Control$ python plantLatJLL.py
'''
import time
import numpy as np

from cereal import log
import cereal.messaging as messaging
from common.realtime import Ratekeeper, DT_MDL  # = 0.05, 20 FPS
from selfdrive.modeld.constants import T_IDXS
  #from selfdrive.controls.lib.lateral_planner import LateralPlanner
from selfdrive.controls.lib.lateral_plannerJLL import LateralPlanner

class Plant:
  def __init__(self):
    self.rate = 1. / DT_MDL
    self.speed = 10.    # car runs at constant speed
    self.distance = 0.  # distance traveled by car

    self.rk = Ratekeeper(self.rate, print_delay_threshold=100.0)
    self.ts = DT_MDL
    time.sleep(1)

    from selfdrive.car.honda.values import CAR
    from selfdrive.car.honda.interface import CarInterface

    self.planner = LateralPlanner(CarInterface.get_non_essential_params(CAR.CIVIC))

  @property
  def current_time(self):
    return float(self.rk.frame) * self.ts

  def step(self):
    control = messaging.new_message('controlsState')
    car_state = messaging.new_message('carState')
    model = messaging.new_message('modelV2')

    # Simulate a fake model going straight at the same self.speed of car
    position = log.XYZTData.new_message()
    position.x = [float(x) for x in self.speed * np.array(T_IDXS)]
    position.y = [float(x) for x in 0. * np.array(T_IDXS)]
    position.z = [float(x) for x in 1.2 * np.array(T_IDXS)]
    position.t = [float(x) for x in np.array(T_IDXS)]
    model.modelV2.position = position

    orientation = log.XYZTData.new_message()
    orientation.z = [float(x) for x in 0. * np.array(T_IDXS)]
    model.modelV2.orientation = orientation

    orientationRate = log.XYZTData.new_message()
    orientationRate.z = [float(x) for x in 0. * np.array(T_IDXS)]
    model.modelV2.orientationRate = orientationRate

    velocity = log.XYZTData.new_message()
    velocity.x = [float(x) for x in self.speed * np.ones_like(T_IDXS)]
    velocity.y = [float(x) for x in 0. * np.ones_like(T_IDXS)]
    velocity.z = [float(x) for x in 0. * np.ones_like(T_IDXS)]
    model.modelV2.velocity = velocity

    # ******** get controlsState messages for plotting ***
    sm = {'carState': car_state.carState,
          'controlsState': control.controlsState,
          'modelV2': model.modelV2}
    self.planner.update(sm)
    self.curvature = self.planner.desired_curvature
    self.curvature_rate = self.planner.desired_curvature_rate

    # print at 5hz
    if (self.rk.frame % (self.rate // 5)) == 0:
      print("%2.2f sec   %6.2f m  %6.2f m/s  %6.2f 1/m  %6.2f 1/m^2"
            % (self.current_time, self.distance, self.speed, self.curvature, self.curvature_rate))
      '''
      0.00 sec     0.00 m   10.00 m/s    0.00 1/m    0.00 1/m^2
      0.20 sec     2.00 m   10.00 m/s    0.00 1/m    0.00 1/m^2
      0.40 sec     4.00 m   10.00 m/s    0.00 1/m    0.00 1/m^2
      9.60 sec    96.00 m   10.00 m/s    0.00 1/m    0.00 1/m^2
      9.80 sec    98.00 m   10.00 m/s    0.00 1/m    0.00 1/m^2
      10.00 sec   100.00 m   10.00 m/s    0.00 1/m    0.00 1/m^2
      '''

    self.distance = self.distance + self.speed * self.ts

    # ******** update prevs ********
    self.rk.monitor_time()

    return {
      "distance": self.distance,
      "speed": self.speed,
    }

# simple engage in standalone mode
def plant_thread():
  plant = Plant()
  while plant.current_time < 10.1:  # seconds
    plant.step()


if __name__ == "__main__":
  plant_thread()
