''' JLL  2023.7.17
for "/cerealJLL"  230714
from /openpilot/controlsdJLL.py

(sconsvenv) jinn@Liu:~/openpilot/aJLL/Control$ python controlsdJLL1.py
'''
from cerealJLL import carJLL
import cerealJLL.messagingJLL as messagingJLL


class Controls:
  def __init__(self, sm=None, pm=None, can_sock=None, CI=None):
    self.joystick_mode = True

    self.sm = sm
    if self.sm is None:
      ignore = ['testJoystick']
      self.sm = messagingJLL.SubMaster(['testJoystick'], ignore_alive=ignore, ignore_avg_freq=['testJoystick'])
        #print('#--- vars(self.sm) =', vars(self.sm))  # Python: Print an Object’s Attributes
        #--- vars(self.sm) = {'data': {'testJoystick': <logJLL.capnp:Joystick builder ()>},
        #---  'valid': {'testJoystick': True}, 'logMonoTime': {'testJoystick': 0}}

  def state_control(self):
    CC = carJLL.CarControl.new_message()
    actuators = CC.actuators
      #print('#--- self.sm[testJoystick] =', self.sm['testJoystick'])
      #--- self.sm[testJoystick] = ()
    self.sm['testJoystick'].axes = [-0.3, 1.]  # = [gas, steer]
    actuators.accel = self.sm['testJoystick'].axes[0]
    actuators.steer = 0.5
      #print('#--- actuators =', actuators)
      #--- actuators = (steer = 0.5, accel = -0.3)
      #--- self.sm[testJoystick] = (axes = [-0.3, 1])
    return CC

  def controlsd_thread(self):
    CC = carJLL.CarControl.new_message()
    CC = self.state_control()
    print('#--- self.sm[testJoystick] =', self.sm['testJoystick'])
      #--- self.sm[testJoystick] = (axes = [-0.3, 1])


def main(sm=None, pm=None, logcan=None):
  controls = Controls(sm, pm, logcan)
  controls.controlsd_thread()


if __name__ == "__main__":
  main()
