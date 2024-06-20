#!/usr/bin/env python3
''' JLL  2023.8.17, 12.13
for 230725 Q1a, 231121
from /openpilot/selfdrive/controls/lib/tests/test_latcontrol.py

(sconsvenv) jinn@Liu:~/openpilot/aJLL/Control$ python test_latcontrolJLL.py
'''
import unittest
from parameterized import parameterized

from cereal import car, log
from selfdrive.car.car_helpers import interfaces
from selfdrive.car.honda.values import CAR as HONDA
from selfdrive.car.toyota.values import CAR as TOYOTA
from selfdrive.car.nissan.values import CAR as NISSAN
from selfdrive.controls.lib.latcontrol_pid import LatControlPID
from selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from selfdrive.controls.lib.latcontrol_indi import LatControlINDI
from selfdrive.controls.lib.latcontrol_angle import LatControlAngle
from selfdrive.controls.lib.vehicle_model import VehicleModel

class TestLatControl(unittest.TestCase):
  #@parameterized.expand([(HONDA.CIVIC, LatControlPID), (TOYOTA.RAV4, LatControlTorque), (TOYOTA.PRIUS, LatControlINDI), (NISSAN.LEAF, LatControlAngle)])
  @parameterized.expand([(HONDA.CIVIC, LatControlPID)])
    #--- CP.lateralTuning.which() = pid
    #--- output_steer, angle_steers_des = -1.0 -3709.497205792661
    #--- FAIL: test_saturation_0_HONDA_CIVIC_2016 (__main__.TestLatControl)
  #@parameterized.expand([(TOYOTA.PRIUS, LatControlPID)])
    #--- ERROR: test_saturation_0_TOYOTA_PRIUS_2017 (__main__.TestLatControl)
    #--- capnp.lib.capnp.KjException: capnp/dynamic.c++:160: failed: expected isSetInUnion(field); Tried to get()
    #--- a union member which is not currently initialized.; ... car.capnp:CarParams.lateralTuning
  #@parameterized.expand([(TOYOTA.PRIUS, LatControlTorque)])
    #--- CP.lateralTuning.which() = torque
    #--- output_steer, angle_steers_des = -1.0 0.0
    #--- FAIL: test_saturation_0_TOYOTA_PRIUS_2017 (__main__.TestLatControl)
  #@parameterized.expand([(TOYOTA.PRIUS, LatControlAngle)])
    #--- CP.lateralTuning.which() = torque
    #--- output_steer, angle_steers_des = 0 -4571.77844337976
    #--- OK
  def test_saturation(self, car_name, controller):
    CarInterface, CarController, CarState = interfaces[car_name]
    CP = CarInterface.get_non_essential_params(car_name)
    CI = CarInterface(CP, CarController, CarState)
    VM = VehicleModel(CP)
    controller = controller(CP, CI)
    print('#--- CP.lateralTuning.which() =', CP.lateralTuning.which())
      #print('#--- vars(VM) =', vars(VM))
      #print('#--- vars(controller) =', vars(controller))
      #--- controller = <selfdrive.controls.lib.latcontrol_angle.LatControlAngle object at 0x7f8844df4250>

    CS = car.CarState.new_message()
    CS.vEgo = 30
      #print('#--- CS =', CS)
    last_actuators = car.CarControl.Actuators.new_message()
      #print('#--- last_actuators =', last_actuators)
    params = log.LiveParametersData.new_message()
      #print('#--- params =', params)

    #for _ in range(1000):
    for _ in range(3):
      output_steer, angle_steers_des, lac_log = controller.update(True, CS, VM, params, last_actuators, True, 1, 0, 0)
    print('#--- output_steer, angle_steers_des =', output_steer, angle_steers_des)
    #self.assertTrue(lac_log.saturated)

if __name__ == "__main__":
  unittest.main()
