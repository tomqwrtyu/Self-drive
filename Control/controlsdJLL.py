''' JLL  2023.2.1
for 230201, 230414, 230419 Q1, 230521 Q1, 230714
from /openpilot/selfdrive/controls/controlsd.py
     /DP091/selfdrive/controls/lib/events.py
with /openpilot/selfdrive/controls/lib/eventsJLL.py

(sconsvenv) jinn@Liu:~/openpilot/aJLL/Control$ python controlsdJLL.py
'''
from cereal import car, log
from common.numpy_fast import clip
  #from selfdrive.controls.lib.events import Events, ET
from selfdrive.controls.lib.eventsJLL import Events, ET, EVENTS
import cereal.messaging as messaging

EventName = car.CarEvent.EventName
ACTUATOR_FIELDS = tuple(car.CarControl.Actuators.schema.fields.keys())
  #--- ACTUATOR_FIELDS = ('gas', 'brake', 'steer', 'steeringAngleDeg', 'accel',
  #                       'longControlState', 'speed', 'curvature', 'steerOutputCan')
State = log.ControlsState.OpenpilotState
ACTIVE_STATES = (State.enabled, State.softDisabling, State.overriding)
  #--- State = <capnp.lib.capnp._EnumModule object at 0x7fd2c54f9ca0>
  #--- ACTIVE_STATES = (2, 3, 4)
ENABLED_STATES = (State.preEnabled, *ACTIVE_STATES)
  #--- ENABLED_STATES = (1, 2, 3, 4)
CSID_MAP = {"1": EventName.roadCameraError, "2": EventName.wideRoadCameraError, "0": EventName.driverCameraError}
  #--- CSID_MAP = {'1': 100, '2': 102, '0': 101}
    # Qualcomm Camera Subsystem driver: 第7讲 Camera Sensor CSID 基本概念   Camera Serial Interface
    # 2 CSIPHY modules. They handle the Physical layer of the CSI2 receivers
    # 2 CSID (CSI Decoder) modules. They handle the Protocol and Application layer of the CSI2 receivers.
    # A CSID can decode data stream from any of the CSIPHY.


class Controls:
  def __init__(self, sm=None, pm=None, can_sock=None, CI=None):
    self.joystick_mode = True
    self.events = Events()

    if self.joystick_mode:
      self.events.add(EventName.joystickDebug, static=True)

    self.sm = sm
    if self.sm is None:
      ignore = ['testJoystick']
      self.sm = messaging.SubMaster(['testJoystick'], ignore_alive=ignore, ignore_avg_freq=['testJoystick'])
        #print('#--- vars(self.sm) =', vars(self.sm))  # Python: Print an Object’s Attributes

  def state_control(self):  # for 230419 Q1
    CC = car.CarControl.new_message()
      #--- CC = ( enabled = false, gasDEPRECATED = 0, brakeDEPRECATED = 0, steeringTorqueDEPRECATED = 0,
      #--- activeDEPRECATED = false, rollDEPRECATED = 0, pitchDEPRECATED = 0, latActive = false,
      #--- longActive = false, leftBlinker = false, rightBlinker = false )
    actuators = CC.actuators
      #--- actuators = (gas = 0, brake = 0, steer = 0, steeringAngleDeg = 0, accel = 0,
      #---  longControlState = off, speed = 0, curvature = 0, steerOutputCan = 0 )
    #self.sm['testJoystick'].axes = [-0.3, 1.]  # = [gas, steer]
    self.sm['testJoystick'].axes = [-0.3, 1., 2., 3.]  # = [gas, steer, c, d] Only 2 componants?
    print('#--- self.sm[testJoystick] =', self.sm['testJoystick'])
      #--- self.sm[testJoystick] = (axes = [-0.3, 1])
      #--- self.sm[testJoystick] = (axes = [-0.3, 1, 2, 3])
      # axes is a List of arbitrary size from "struct Joystick {axes @0: List(Float32);}" in log.capnp
    actuators.accel = self.sm['testJoystick'].axes[0]
    steer = clip(self.sm['testJoystick'].axes[1], -1, 1)
      # max angle is 45 for angle-based cars
    actuators.steer, actuators.steeringAngleDeg = steer, steer * 45.
    print('#--- actuators =', actuators)  # for 230521 Q1
    print('#--- CC.actuators =', CC.actuators)  # Q1 > A: Yes, actuators == CC.actuators
      #--- CC.actuators = (gas = 0, brake = 0, steer = 1, steeringAngleDeg = 45, accel = -0.3,
    return CC

  def controlsd_thread(self):
    evs = EVENTS[self.events.names[0]].values()
      #--- self.events.names = [34]  # from joystickDebug @34; by lib.eventsJLL
      #--- self.events.events = [34]
      #--- self.events.static_events = [34]
      #--- self.events.events_prev = {34: 0}
      #--- EVENTS[self.events.names[0]].keys() = dict_keys(['warning', 'permanent'])
    list(evs)[0]()  # calls joystick_alert() in lib.eventsJLL. who calls this in DP091?
      #--- list(evs)[0] = <function joystick_alert at 0x7f505b671820>
    ees = (EVENTS.get(k, {}) for k in self.events.events)
      #--- self.events.events_prev = {34: 0}
      #--- EventName.joystickDebug = 34
      #print('#--- any() =', any(EVENTS.get(k, {}) for k in self.events.events))
      #--- any() = True
    for value in ees:
      eesvl = value
      #--- eesvl.keys() = dict_keys(['warning', 'permanent'])
    list(eesvl.values())[0]()  # calls joystick_alert() in lib.eventsJLL. who calls this in DP091?

    CS = car.CarState.new_message()
      #--- CS = ( vEgo = 0, gas = 0, gasPressed = false, brake = 0, brakePressed = false, steeringAngleDeg = 0,
    CC = car.CarControl.new_message()
      #--- CC.actuators.accel = 0.0  # for 230521 Q1
    CC = self.state_control()  # for 230419 Q1
      #--- CC.actuators.accel = -0.3  # for 230521 Q1


def main(sm=None, pm=None, logcan=None):
  controls = Controls(sm, pm, logcan)
  controls.controlsd_thread()


if __name__ == "__main__":
  main()
