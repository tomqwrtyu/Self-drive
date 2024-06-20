CAN 10c   JLL, 2021.4.21-4.27, 6.14-6.22, 30
Code Overview: From Neural Net to CAN by Steering
--------------------------------------------------

---------- Step 3: Controls
----- Input 1: sub_sock('can'), carState (CS)
----- Input 2: sm['pathPlan']
----- Output 1: pm.send('sendcan', canData)
----- Output 2: pm.send('carControl'(CC), 'controlsState', 'carState', ...)
----- Return: Input 1: sub_sock
----- Return: Output 1: steerAngle, CANPacker, canData
----- Return: Output 1: CANPacker: CI.apply.CC.update.create_steer_command.packer.make_can_msg.pyx.pack.cc.pack
----- Return: Output 1: canData: pm.send.py.pyx.cpp.can_list_to_can_capnp_cpp(canData).toBytes.messageToFlatArray.asBytes
----- Keywords: canData, capnp, new_message()

----- Controls, steerAngle
/OP081/selfdrive/controls/controlsd.py
/OP081/selfdrive/controls/lib/latcontrol_indi.py
/OP081/selfdrive/controls/lib/vehicle_model.py
/OP081/selfdrive/controls/lib/drive_helpers.py
/OP081/selfdrive/car/car_helpers.py
/OP081/selfdrive/car/toyota/interface.py
/OP081/selfdrive/car/toyota/carstate.py
/OP081/selfdrive/car/interfaces.py
/OP081/selfdrive/car/toyota/carcontroller.py
/OP081/selfdrive/car/__init__.py
/OP081/selfdrive/car/toyota/toyotacan.py

----- CANPacker
/OP081/opendbc/can/packer.py
/OP081/opendbc/can/packer_pyx.pyx
/OP081/opendbc/can/common.pxd
/OP081/opendbc/can/packer_pyx.so
/OP081/opendbc/can/common_dbc.h
/OP081/opendbc/can/common.h
/OP081/opendbc/can/packer.cc
/OP081/opendbc/can/SConscript

----- canData
/OP081/selfdrive/boardd/boardd.py
/OP081/selfdrive/boardd/boardd_api_impl.pyx
/OP081/selfdrive/boardd/boardd_api_impl.so
/OP081/selfdrive/boardd/boardd_api_impl.cpp
/OP081/selfdrive/boardd/can_list_to_can_capnp.cc
/OP081/selfdrive/boardd/SConscript
/OP081/cereal/messaging/messaging.hpp

----- Messaging in zcCS3d
/OP081/cereal/messaging/__init__.py
/OP081/cereal/__init__.py
/OP081/cereal/car.capnp
/OP081/cereal/log.capnp
/OP081/cereal/include/c++.capnp
/OP081/cereal/messaging/messaging_pyx.pyx
/OP081/cereal/messaging/messaging.pxd
/OP081/cereal/messaging/messaging.hpp
/OP081/cereal/messaging/messaging.cc
/OP081/cereal/messaging/impl_zmq.hpp
/OP081/cereal/messaging/impl_zmq.cc
/OP081/cereal/services.h
--------------------------------------------------

---------- Step 3: Controls
----- Controls, steerAngle

/OP081/selfdrive/controls/controlsd.py
  import cereal.messaging as messaging
  from cereal import car, log
  from selfdrive.boardd.boardd import can_list_to_can_capnp
  from selfdrive.car.car_helpers import get_car, get_one_can
  from selfdrive.controls.lib.latcontrol_indi import LatControlINDI
  from selfdrive.controls.lib.vehicle_model import VehicleModel
  EventName = car.CarEvent.EventName
  class Controls:
    def __init__(self, sm=None, pm=None, can_sock=None):
      self.pm = messaging.PubMaster(['sendcan', 'controlsState', 'carState',
                                     'carControl', 'carEvents', 'carParams'])
      self.sm = messaging.SubMaster([, ... , 'pathPlan', ])
----- Input 1: sub_sock('can').can_sock
      self.can_sock = messaging.sub_sock('can', timeout=can_timeout)
      print("Waiting for CAN messages...")
      get_one_can(self.can_sock)
      self.CI, self.CP = get_car(self.can_sock, self.pm.sock['sendcan'])
      self.CC = car.CarControl.new_message()
      self.VM = VehicleModel(self.CP)
      elif self.CP.lateralTuning.which() == 'indi':
        self.LaC = LatControlINDI(self.CP)
    def update_events(self, CS):
      # Compute carEvents from carState
      self.events.add_from_msg(CS.events)
----- Input 1: carState (CS)
    def data_sample(self):
      # Receive data from sockets and update carState from CAN
      # Update carState from CAN
      can_strs = messaging.drain_sock_raw(self.can_sock, wait_for_one=True)
----- Return: Input 1: CI.update.CS
      CS = self.CI.update(self.CC, can_strs)
      self.distance_traveled += CS.vEgo * DT_CTRL
      return CS
----- Return: Output 1: steerAngle: state_control(CS).path_plan.LaC.update
    def state_control(self, CS):
      # Given the state, this function returns an actuators packet"""
----- Input 2: sm['pathPlan']
      path_plan = self.sm['pathPlan']
      actuators = car.CarControl.Actuators.new_message()
      # Steering PID loop and lateral MPC
      actuators.steer, actuators.steerAngle, lac_log = self.LaC.update(, CS, self.CP, path_plan, )
      return actuators, v_acc_sol, a_acc_sol, lac_log
    def publish_logs(self, CS, start_time, actuators, v_acc, a_acc, lac_log):
      # Send actuators and hud commands to the car, send controlsstate and MPC logging"""
      CC = car.CarControl.new_message()
      CC.actuators = actuators
      steer_angle_rad = (CS.steeringAngle - self.sm['pathPlan'].angleOffset) * CV.DEG_TO_RAD
      dat = messaging.new_message('controlsState')
      controlsState = dat.controlsState
      controlsState.vEgo = CS.vEgo
      controlsState.angleSteers = CS.steeringAngle
      controlsState.curvature = self.VM.calc_curvature(steer_angle_rad, CS.vEgo)
      controlsState.angleSteersDes = float(self.LaC.angle_steers_des)
        elif self.CP.lateralTuning.which() == 'indi':
          controlsState.lateralControlState.indiState = lac_log
      cs_send = messaging.new_message('carState')
      ce_send = messaging.new_message('carEvents', len(self.events))
      cp_send = messaging.new_message('carParams')
      cc_send = messaging.new_message('carControl')
----- Return: Output 1: CANPacker: CI.apply.CC
      can_sends = self.CI.apply(CC) # send car controls over can
----- Return: Output 1: canData: pm.send.can_list_to_can_capnp(can_sends)
----- Output 1: pm.send('sendcan', canData)
      self.pm.send('sendcan', can_list_to_can_capnp(can_sends, msgtype='sendcan', valid=CS.canValid))
----- Output 2: 'carControl', 'controlsState', 'carState', ...
      self.pm.send('controlsState', dat)
      self.pm.send('carState', cs_send)
      self.pm.send('carEvents', ce_send)
      cc_send.carControl = CC
      self.pm.send('carControl', cc_send)
----- Return: Output 1: steerAngle: step.state_control.path_plan.LaC.update
    def step(self):
      # Sample data from sockets and get a carState
      CS = self.data_sample()
      self.update_events(CS)
      # Compute actuators (runs PID loops and lateral MPC)
      actuators, v_acc, a_acc, lac_log = self.state_control(CS)
      self.publish_logs(CS, start_time, actuators, v_acc, a_acc, lac_log)
    def controlsd_thread(self):
      while True:
        self.step()
  def main(sm=None, pm=None, logcan=None):
    controls = Controls(sm, pm, logcan)
    controls.controlsd_thread()
  if __name__ == "__main__":
    main()

--- car.CarControl.new_message() => capnp.new_message(): Initialize a New Cap’n Proto Object
      addresses = addressbook_capnp.AddressBook.new_message()
    https://capnproto.github.io/pycapnp/quickstart.html
    https://capnproto.org/language.html
--- Should data_sample(): be defined before update_events(, CS):???
    CS is defined in data_sample() that is called before update_events() in step().
--- a thread of execution is the smallest sequence of programmed instructions

/OP081/selfdrive/controls/lib/latcontrol_indi.py
  from selfdrive.car import apply_toyota_steer_torque_limits
  from selfdrive.controls.lib.drive_helpers import get_steer_max
  class LatControlINDI():
    def __init__(self, CP):
      A = np.array([[1.0, DT_CTRL, 0.0],
      C = np.array([[1.0, 0.0, 0.0],
      K = np.array([[7.30262179e-01, 2.07003658e-04],
      self.K = K
      self.A_K = A - np.dot(K, C)
----- Return: Output 1: steerAngle: step.state_control.LaC.update.get_steer_max
    def update(self, active, CS, CP, VM, params, lat_plan):
      # Update Kalman filter
      y = np.array([[math.radians(CS.steeringAngle)], [math.radians(CS.steeringRate)]])
      self.x = np.dot(self.A_K, self.x) + np.dot(self.K, y)
      indi_log.steerAngle = math.degrees(self.x[0])
      self.angle_steers_des = path_plan.angleSteers
      steers_des = math.radians(self.angle_steers_des)
      new_output_steer_cmd = apply_toyota_steer_torque_limits(new_output_steer_cmd, prev_output_steer_cmd, prev_output_steer_cmd, CarControllerParams)
      self.output_steer = new_output_steer_cmd / steer_max
      steers_max = get_steer_max(CP, CS.vEgo)
      self.output_steer = clip(self.output_steer, -steers_max, steers_max)
      indi_log.output = float(self.output_steer)
      return float(self.output_steer), float(self.angle_steers_des), indi_log

/OP081/selfdrive/controls/lib/vehicle_model.py
  """
  Dynamic bycicle model from "The Science of Vehicle Dynamics (2014), M. Guiggiani"
  The state is x = [v, r]^T
    with v lateral speed [m/s], and r rotational speed [rad/s]
  The input u is the steering angle [rad]
  The system is defined by
    x_dot = A*x + B*u
  A depends on longitudinal speed, u [m/s], and vehicle parameters CP
  """
  class VehicleModel:
    def __init__(self, CP: car.CarParams):
      # for math readability, convert long names car params into short names
      self.m = CP.mass
      self.l = CP.wheelbase
      self.aF = CP.centerToFront
      self.aR = CP.wheelbase - CP.centerToFront
      self.chi = CP.steerRatioRear
      self.cF_orig = CP.tireStiffnessFront
      self.cR_orig = CP.tireStiffnessRear
      self.update_params(1.0, CP.steerRatio)
    def update_params(self, stiffness_factor: float, steer_ratio: float) -> None:
      # Update the vehicle model with a new stiffness factor and steer ratio
      self.cF = stiffness_factor * self.cF_orig
      self.cR = stiffness_factor * self.cR_orig
      self.sR = steer_ratio
    def calc_curvature(self, sa: float, u: float) -> float:
      # Returns the curvature. Multiplied by the speed this will give the yaw rate.
      # Args: sa: Steering wheel angle [rad], u: Speed [m/s]
      # Returns: Curvature factor [1/m]
      return self.curvature_factor(u) * sa / self.sR
    def curvature_factor(self, u: float) -> float:
      # Multiplied by wheel angle (not steering wheel angle) this will give the curvature.
      # Args: u: Speed [m/s]
      # Returns: Curvature factor [1/m]
      sf = calc_slip_factor(self)
      return (1. - self.chi) / (1. - sf * u**2) / self.l
  def calc_slip_factor(VM):
    # The slip factor is a measure of how the curvature changes with speed
    #   it's positive for Oversteering vehicle, negative (usual case) otherwise.
    return VM.m * (VM.cF * VM.aF - VM.cR * VM.aR) / (VM.l**2 * VM.cF * VM.cR)

/OP081/selfdrive/controls/lib/drive_helpers.py
  from common.numpy_fast import clip, interp
----- Return: Output 1: steerAngle: step.state_control.LaC.update.get_steer_max.steerMaxBP
  def get_steer_max(CP, v_ego):
    return interp(v_ego, CP.steerMaxBP, CP.steerMaxV)

/OP081/selfdrive/car/car_helpers.py
  import cereal.messaging as messaging
  def get_one_can(logcan):
    while True:
      can = messaging.recv_one_retry(logcan)
      if len(can.can) > 0:
        return can
  def load_interfaces(brand_names):
      path = ('selfdrive.car.%s' % brand_name)
      CarInterface = __import__(path + '.interface', fromlist=['CarInterface']).CarInterface
      CarState = __import__(path + '.carstate', fromlist=['CarState']).CarState
      CarController = __import__(path + '.carcontroller', fromlist=['CarController']).CarController
      ret[model_name] = (CarInterface, CarController, CarState)
    return ret
  interfaces = load_interfaces(interface_names)
  def only_toyota_left(candidate_cars):
  def fingerprint(logcan, sendcan):
  def get_car(logcan, sendcan):
    candidate, fingerprints, vin, car_fw, source = fingerprint(logcan, sendcan)
    CarInterface, CarController, CarState = interfaces[candidate]
    car_params = CarInterface.get_params(candidate, fingerprints, car_fw)
    return CarInterface(car_params, CarController, CarState), car_params

/OP081/selfdrive/car/toyota/interface.py
  from cereal import car
  from selfdrive.car.toyota.values import Ecu, ECU_FINGERPRINT, CAR, TSS2_CAR, FINGERPRINTS
  from selfdrive.car.interfaces import CarInterfaceBase
  class CarInterface(CarInterfaceBase):
      if candidate == CAR.PRIUS:
        ret.lateralTuning.init('indi')
    # returns a car.CarState
----- Return: Input 1: CI.update.CS.update
    def update(self, c, can_strings):
      # ******************* do can recv *******************
      ret = self.CS.update(self.cp, self.cp_cam)
      events = self.create_common_events(ret)
      if self.CS.low_speed_lockout and self.CP.openpilotLongitudinalControl:
        events.add(EventName.lowSpeedLockout)
      if ret.vEgo < self.CP.minEnableSpeed and self.CP.openpilotLongitudinalControl:
        events.add(EventName.belowEngageSpeed)
        if c.actuators.gas > 0.1:
          # some margin on the actuator to not false trigger cancellation while stopping
          events.add(EventName.speedTooLow)
        if ret.vEgo < 0.001:
          # while in standstill, send a user alert
          events.add(EventName.manualRestart)
      ret.events = events.to_msg()
      self.CS.out = ret.as_reader()
      return self.CS.out
    # pass in a car.CarControl to be called @ 100hz
----- Return: Output 1: CANPacker: CI.apply.CC.update(CS)
    def apply(self, c):
      can_sends = self.CC.update(c.enabled, self.CS, self.frame, c.actuators, , , , c.hudControl.leftLaneVisible,
                               c.hudControl.rightLaneVisible, c.hudControl.leadVisible,, )
      self.frame += 1
      return can_sends

/OP081/selfdrive/car/toyota/carstate.py
  from cereal import car
  from opendbc.can.can_define import CANDefine
  from selfdrive.car.interfaces import CarStateBase
  from opendbc.can.parser import CANParser
  from selfdrive.car.toyota.values import CAR, DBC, STEER_THRESHOLD, TSS2_CAR, NO_STOP_TIMER_CAR
  class CarState(CarStateBase):
    def __init__(self, CP):
      super().__init__(CP)
      can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
      # All TSS2 car have the accurate sensor
      self.accurate_steer_angle_seen = CP.carFingerprint in TSS2_CAR
      # On NO_DSU cars but not TSS2 cars the cp.vl["STEER_TORQUE_SENSOR"]['STEER_ANGLE']
      # is zeroed to where the steering angle is at start.
      # Need to apply an offset as soon as the steering angle measurements are both received
      self.needs_angle_offset = CP.carFingerprint not in TSS2_CAR
      self.angle_offset = 0.
----- Return: Input 1: CI.update.CS.update.update_speed_kf
    def update(self, cp, cp_cam):
      ret = car.CarState.new_message()
      if self.CP.enableGasInterceptor:
      ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
      ret.standstill = ret.vEgoRaw < 0.001
      # Some newer models have a more accurate angle measurement in the TORQUE_SENSOR message. Use if non-zero
      if abs(cp.vl["STEER_TORQUE_SENSOR"]['STEER_ANGLE']) > 1e-3:
        self.accurate_steer_angle_seen = True
      if self.accurate_steer_angle_seen:
        ret.steeringAngle = cp.vl["STEER_TORQUE_SENSOR"]['STEER_ANGLE'] - self.angle_offset
      if self.CP.carFingerprint in NO_STOP_TIMER_CAR or self.CP.enableGasInterceptor:
        # ignore standstill in hybrid vehicles, since pcm allows to restart without
        # receiving any special command. Also if interceptor is detected
        ret.cruiseState.standstill = False
      return ret
    @staticmethod
    def get_can_parser(CP):
      signals = [
        # sig_name, sig_address, default
        ("STEER_ANGLE", "STEER_ANGLE_SENSOR", 0),
        ("STEER_ANGLE", "STEER_TORQUE_SENSOR", 0),]
      checks = [
        ("STEER_ANGLE_SENSOR", 80),]
      return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

--- cp.vl: cp defined in interfaces.py, vl in STATIC_MSGS in carcontroller.py and
      values.py

/OP081/selfdrive/car/interfaces.py
  class CarInterfaceBase():
    def __init__(self, CP, CarController, CarState):
      self.CP = CP
      self.VM = VehicleModel(CP)
      self.frame = 0
      self.CS = CarState(CP)
      self.cp = self.CS.get_can_parser(CP)
      self.cp_cam = self.CS.get_cam_can_parser(CP)
      self.cp_body = self.CS.get_body_can_parser(CP)
      self.CC = CarController(self.cp.dbc_name, CP, self.VM)
----- Return: Input 1: CI.update.CS.update.update_speed_kf.v_ego_x
  def update_speed_kf(self, v_ego_raw):
    if abs(v_ego_raw - self.v_ego_kf.x[0][0]) > 2.0:  # Prevent large accelerations when car starts at non zero speed
      self.v_ego_kf.x = [[v_ego_raw], [0.0]]
    v_ego_x = self.v_ego_kf.update(v_ego_raw)
    return float(v_ego_x[0]), float(v_ego_x[1])

/OP081/selfdrive/car/toyota/carcontroller.py
  from selfdrive.car import apply_toyota_steer_torque_limits, create_gas_command, make_can_msg
  from selfdrive.car.toyota.toyotacan import create_steer_command, create_ui_command, \
  from selfdrive.car.toyota.values import Ecu, CAR, STATIC_MSGS, NO_STOP_TIMER_CAR, SteerLimitParams
  from opendbc.can.packer import CANPacker
  class CarController()
    def __init__(self, dbc_name, CP, VM):
      self.last_steer = 0
      self.steer_rate_limited = False
      self.fake_ecus = set()
      if CP.enableCamera:
        self.fake_ecus.add(Ecu.fwdCamera)
      if CP.enableDsu:
        self.fake_ecus.add(Ecu.dsu)
      self.packer = CANPacker(dbc_name)
----- Return: Output 1: CANPacker: CI.apply.CC.update.create_steer_command(packer)
                        CANPacker: CI.apply.CC.update.make_can_msg(STATIC_MSGS)
    def update(self, enabled, CS, frame, actuators, pcm_cancel_cmd, hud_alert,
      # steer torque
      new_steer = int(round(actuators.steer * SteerLimitParams.STEER_MAX))
      apply_steer = apply_toyota_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, SteerLimitParams)
      self.steer_rate_limited = new_steer != apply_steer
      # Cut steering while we're in a known fault state (2s)
      if not enabled or CS.steer_state in [9, 25]:
        apply_steer = 0
        apply_steer_req = 0
      else:
        apply_steer_req = 1
      self.last_steer = apply_steer
      #*** control msgs ***
      #print("steer {0} {1} {2} {3}".format(apply_steer, min_lim, max_lim, CS.steer_torque_motor)
      # toyota can trace shows this message at 42Hz, with counter adding alternatively 1 and 2;
      # sending it at 100Hz seem to allow a higher rate limit, as the rate limit seems imposed
      # on consecutive messages
      can_sends = []
      if Ecu.fwdCamera in self.fake_ecus:
        can_sends.append(create_steer_command(self.packer, apply_steer, apply_steer_req, frame))
      #*** static msgs ***
      for (addr, ecu, cars, bus, fr_step, vl) in STATIC_MSGS:
        if frame % fr_step == 0 and ecu in self.fake_ecus and CS.CP.carFingerprint in cars:
          can_sends.append(make_can_msg(addr, vl, bus))
      return can_sends

/OP081/selfdrive/car/__init__.py
  # functions common among cars
  from common.numpy_fast import clip
  def gen_empty_fingerprint():
  def dbc_dict(pt_dbc, radar_dbc, chassis_dbc=None, body_dbc=None):
    return {'pt': pt_dbc, 'radar': radar_dbc, 'chassis': chassis_dbc, 'body': body_dbc}
  def apply_std_steer_torque_limits(apply_torque, apply_torque_last, driver_torque, LIMITS):
----- Return: Output 1: steerAngle: step.state_control.LaC.update.apply_toyota_steer_torque_limits.apply_torque
  def apply_toyota_steer_torque_limits(apply_torque, apply_torque_last, motor_torque, LIMITS):
    apply_torque = clip(apply_torque, min_lim, max_lim)
    return int(round(float(apply_torque)))
----- Return: Output 1: CANPacker: CI.apply.CC.update.make_can_msg.dat
  def make_can_msg(addr, dat, bus):
    return [addr, 0, dat, bus]

/OP081/selfdrive/car/toyota/toyotacan.py
----- Return: Output 1: CANPacker: CI.apply.CC.update.create_steer_command.packer.make_can_msg
  def create_steer_command(packer, steer, steer_req, raw_cnt):
    # Creates a CAN message for the Toyota Steer Command.
    values = {
      "STEER_TORQUE_CMD": steer,
    return packer.make_can_msg("STEERING_LKA", 0, values)

----- CANPacker

/OP081/opendbc/can/packer.py
  from opendbc.can.packer_pyx import CANPacker
  assert CANPacker

/OP081/opendbc/can/packer_pyx.pyx
  # distutils: language = c++
  # cython: c_string_encoding=ascii, language_level=3
  from .common cimport CANPacker as cpp_CANPacker
  from .common cimport dbc_lookup, SignalPackValue, DBC
  cdef class CANPacker:
    cdef:
      cpp_CANPacker *packer
      const DBC *dbc
      map[string, (int, int)] name_to_address_and_size
      map[int, int] address_to_size
    def __init__(self, dbc_name):
      self.dbc = dbc_lookup(dbc_name)
      self.packer = new cpp_CANPacker(dbc_name)
      num_msgs = self.dbc[0].num_msgs
      for i in range(num_msgs):
        msg = self.dbc[0].msgs[i]
        self.name_to_address_and_size[string(msg.name)] = (msg.address, msg.size)
        self.address_to_size[msg.address] = msg.size
    cdef uint64_t pack(self, addr, values, counter):
      cdef vector[SignalPackValue] values_thing
      cdef SignalPackValue spv
      names = []
      for name, value in values.iteritems():
        n = name.encode('utf8')
        names.append(n) # TODO: find better way to keep reference to temp string around
        spv.name = n
        spv.value = value
        values_thing.push_back(spv)
      return self.packer.pack(addr, values_thing, counter)
    cdef inline uint64_t ReverseBytes(self, uint64_t x):
      return (((x & 0xff00000000000000ull) >> 56) |
             ((x & 0x00ff000000000000ull) >> 40) |
----- Return: Output 1: CANPacker: CI.apply.CC.update.create_steer_command.packer.make_can_msg.pyx.pack
    cpdef make_can_msg(self, name_or_addr, bus, values, counter=-1):
      cdef int addr, size
      if type(name_or_addr) == int:
        addr = name_or_addr
        size = self.address_to_size[name_or_addr]
      else:
        addr, size = self.name_to_address_and_size[name_or_addr.encode('utf8')]
      cdef uint64_t val = self.pack(addr, values, counter)
      val = self.ReverseBytes(val)
      return [addr, 0, (<char *>&val)[:size], bus]

/OP081/opendbc/can/common.pxd
  # distutils: language = c++
  #cython: language_level=3
  cdef extern from "common_dbc.h":
    cdef struct DBC:
      const char* name
      size_t num_msgs
      const Msg *msgs
      const Val *vals
      size_t num_vals
    cdef struct SignalPackValue:
      const char * name
      double value
  cdef extern from "common.h":
    cdef const DBC* dbc_lookup(const string);
    cdef cppclass CANParser:
    cdef cppclass CANPacker:
     CANPacker(string)
     uint64_t pack(uint32_t, vector[SignalPackValue], int counter)

/OP081/opendbc/can/packer_pyx.so

/OP081/opendbc/can/common_dbc.h
  struct SignalPackValue {
  struct SignalValue {
    uint32_t address;
    uint16_t ts;
    const char* name;
    double value;};
  enum SignalType {
    TOYOTA_CHECKSUM,
  struct Signal {
    const char* name;
    int b1, b2, bo;
    bool is_signed;
    double factor, offset;
    bool is_little_endian;
    SignalType type;};
  struct Msg {
    const char* name;
    uint32_t address;
    unsigned int size;
    size_t num_sigs;
    const Signal *sigs;};
  struct Val {
    const char* name;
    uint32_t address;
    const char* def_val;
    const Signal *sigs;};
  struct DBC {
    const char* name;
    size_t num_msgs;
    const Msg *msgs;
    const Val *vals;
    size_t num_vals;};
  const DBC* dbc_lookup(const std::string& dbc_name);
  void dbc_register(const DBC* dbc);
  #define dbc_init(dbc) \
  static void __attribute__((constructor)) do_dbc_init_ ## dbc(void) { \
    dbc_register(&dbc); \

/OP081/opendbc/can/common.h
  #include "common_dbc.h"
  #include <capnp/serialize.h>
  #include "cereal/gen/cpp/log.capnp.h"
  unsigned int toyota_checksum(unsigned int address, uint64_t d, int l);
  class MessageState {
  class CANParser {
  class CANPacker {
  private:
    const DBC *dbc = NULL;
    std::map<std::pair<uint32_t, std::string>, Signal> signal_lookup;
    std::map<uint32_t, Msg> message_lookup;
  public:
    CANPacker(const std::string& dbc_name);
    uint64_t pack(uint32_t address, const std::vector<SignalPackValue> &signals, int counter);

/OP081/opendbc/can/packer.cc
  #include "common.h"
  // this is the same as read_u64_le, but uses uint64_t as in/out
  uint64_t ReverseBytes(uint64_t x) {
    return ((x & 0xff00000000000000ull) >> 56) |
            ((x & 0x00ff000000000000ull) >> 40) |
  uint64_t set_value(uint64_t ret, Signal sig, int64_t ival){
    int shift = sig.is_little_endian? sig.b1 : sig.bo;
    uint64_t mask = ((1ULL << sig.b2)-1) << shift;
    uint64_t dat = (ival & ((1ULL << sig.b2)-1)) << shift;
    if (sig.is_little_endian) {
      dat = ReverseBytes(dat);
      mask = ReverseBytes(mask);}
    ret &= ~mask;
    ret |= dat;
    return ret;}
  CANPacker::CANPacker(const std::string& dbc_name) {
    dbc = dbc_lookup(dbc_name);
    assert(dbc);
    for (int i=0; i<dbc->num_msgs; i++) {
      const Msg* msg = &dbc->msgs[i];
      message_lookup[msg->address] = *msg;
      for (int j=0; j<msg->num_sigs; j++) {
        const Signal* sig = &msg->sigs[j];
        signal_lookup[std::make_pair(msg->address, std::string(sig->name))] = *sig;
    init_crc_lookup_tables();}
----- Return: Output 1: CANPacker: CI.apply.CC.update.create_steer_command.packer.make_can_msg.pyx.pack.cc.pack.ret
  uint64_t CANPacker::pack(uint32_t address, const std::vector<SignalPackValue> &signals, int counter) {
    uint64_t ret = 0;
    for (const auto& sigval : signals) {
      std::string name = std::string(sigval.name);
      double value = sigval.value;
      auto sig_it = signal_lookup.find(std::make_pair(address, name));
      auto sig = sig_it->second;
      int64_t ival = (int64_t)(round((value - sig.offset) / sig.factor));
      if (ival < 0) {
        ival = (1ULL << sig.b2) + ival;}
      ret = set_value(ret, sig, ival);}
    if (counter >= 0){
      auto sig_it = signal_lookup.find(std::make_pair(address, "COUNTER"));
      auto sig = sig_it->second;
      ret = set_value(ret, sig, counter);}
    auto sig_it_checksum = signal_lookup.find(std::make_pair(address, "CHECKSUM"));
    return ret;}

/OP081/opendbc/can/SConscript
  Import('env', 'envCython', 'cereal')
  libdbc = env.SharedLibrary('libdbc', ["dbc.cc", "parser.cc", "packer.cc", "common.cc"]+dbcs, LIBS=["capnp", "kj"])
  # Build packer and parser
  lenv = envCython.Clone()
  lenv["LINKFLAGS"] += [libdbc[0].get_labspath()]
  lenv.Program('parser_pyx.so', 'parser_pyx.pyx')
  lenv.Program('packer_pyx.so', 'packer_pyx.pyx')

----- canData

/OP081/selfdrive/boardd/boardd.py
----- Return: Output 1: canData: pm.send.py.can_list_to_can_capnp
  from selfdrive.boardd.boardd_api_impl import can_list_to_can_capnp
  assert can_list_to_can_capnp

/OP081/selfdrive/boardd/boardd_api_impl.pyx
  # distutils: language = c++
  # cython: language_level=3
  from libcpp.vector cimport vector
  from libcpp.string cimport string
  from libcpp cimport bool
  cdef struct can_frame:
    long address
    string dat
    long busTime
    long src
    cdef extern void can_list_to_can_capnp_cpp(const vector[can_frame] &can_list, string &out, bool sendCan, bool valid)
----- Return: Output 1: canData: pm.send.py.pyx.can_list_to_can_capnp_cpp(,,'sendcan',)
  def can_list_to_can_capnp(can_msgs, msgtype='can', valid=True):
    cdef vector[can_frame] can_list
    cdef can_frame f
    for can_msg in can_msgs:
      f.address = can_msg[0]
      f.busTime = can_msg[1]
      f.dat = can_msg[2]
      f.src = can_msg[3]
      can_list.push_back(f)
    cdef string out
    can_list_to_can_capnp_cpp(can_list, out, msgtype == 'sendcan', valid)
    return out

/OP081/selfdrive/boardd/boardd_api_impl.so

/OP081/selfdrive/boardd/boardd_api_impl.cpp
  /* Generated by Cython 0.29.20 */
  #include "Python.h"
  /* Module declarations from 'selfdrive.boardd.boardd_api_impl' */
----- Return: Output 1: canData: pm.send.py.pyx.cpp.can_list_to_can_capnp_cpp( &, &, , )
  __PYX_EXTERN_C DL_IMPORT(void) can_list_to_can_capnp_cpp(std::vector<struct __pyx_t_9selfdrive_6boardd_15boardd_api_impl_can_frame>  const &, std::string &, bool, bool); /*proto*/
  #define __Pyx_MODULE_NAME "selfdrive.boardd.boardd_api_impl"
  extern int __pyx_module_is_main_selfdrive__boardd__boardd_api_impl;
  int __pyx_module_is_main_selfdrive__boardd__boardd_api_impl = 0;
  /* Implementation of 'selfdrive.boardd.boardd_api_impl' */
  static const char __pyx_k_f[] = "f";
  static const char __pyx_k_can[] = "can";

/OP081/selfdrive/boardd/can_list_to_can_capnp.cc
  #include "messaging.hpp"
  typedef struct {
  	long address;
  	std::string dat;
  	long busTime;
  	long src;} can_frame;
  extern "C" {
----- Return: Output 1: canData: pm.send.py.pyx.cpp.can_list_to_can_capnp_cpp(&can_list, &out==bytes==msg==canData,,).msg.toBytes()
  void can_list_to_can_capnp_cpp(const std::vector<can_frame> &can_list, std::string &out, bool sendCan, bool valid) {
    MessageBuilder msg;
    auto event = msg.initEvent(valid); // event is msg, a placeholder holding a copy of msg.
    auto canData = sendCan ? event.initSendcan(can_list.size()) : event.initCan(can_list.size()); // canData is event.
    int j = 0; // initSendcan() is defined in log.capnp.h
    for (auto it = can_list.begin(); it != can_list.end(); it++, j++) {
      auto c = canData[j]; // c is canData[j].
      c.setAddress(it->address);
      c.setBusTime(it->busTime);
      c.setDat(kj::arrayPtr((uint8_t*)it->dat.data(), it->dat.size()));
      c.setSrc(it->src);}
    auto bytes = msg.toBytes();
    out.append((const char *)bytes.begin(), bytes.size());}}

/OP081/selfdrive/boardd/SConscript
  Import('env', 'envCython', 'common', 'cereal', 'messaging')
  env.Program('boardd', ['boardd.cc', 'panda.cc', 'pigeon.cc'], LIBS=['usb-1.0', common, cereal, messaging, 'pthread', 'zmq', 'capnp', 'kj'])
  env.Library('libcan_list_to_can_capnp', ['can_list_to_can_capnp.cc'])
  envCython.Program('boardd_api_impl.so', 'boardd_api_impl.pyx', LIBS=["can_list_to_can_capnp", 'capnp', 'kj'] + envCython["LIBS"])

/OP081/cereal/messaging/messaging.hpp
  #include <capnp/serialize.h>
  class MessageBuilder : public capnp::MallocMessageBuilder {
    cereal::Event::Builder initEvent(bool valid = true) {
      cereal::Event::Builder event = initRoot<cereal::Event>();
      return event;}
----- Return: Output 1: canData: pm.send.py.pyx.cpp.can_list_to_can_capnp_cpp(canData).toBytes.messageToFlatArray.asBytes
    kj::ArrayPtr<capnp::byte> toBytes() {
      heapArray_ = capnp::messageToFlatArray(*this);
      return heapArray_.asBytes();

--- see zcCS3a
