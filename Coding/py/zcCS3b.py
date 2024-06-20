CAN 10b   JLL, 2021.6.15-6.22, 29
Code Overview: From Neural Net to CAN by Steering
--------------------------------------------------

---------- Step 2: Planner
----- Input: ModelData: LP.parse_model.model_polyfit(,)
----- Run MPC: libmpc_py.libmpc.c.run_mpc.acado_getNWSR()
----- Output: PathPlanner.update.pm.send."plan_send".messaging.new_message('pathPlan')
----- Return: Output: PathPlanner.update.pm.send."PathPlan".messaging.new_message('pathPlan').log.capnp.new_message
----- Keywords: PathPlan, capnp, new_message(), mpc, cffi, acado

/OP081/selfdrive/controls/plannerd.py
/OP081/selfdrive/controls/lib/pathplanner.py
/OP081/selfdrive/controls/lib/lane_planner.py
/OP081/selfdrive/controls/lib/lateral_mpc/libmpc_py.py
/OP081/selfdrive/controls/lib/lateral_mpc/libmpc.so
/OP081/selfdrive/controls/lib/lateral_mpc/lateral_mpc.c
/OP081/selfdrive/controls/lib/lateral_mpc/SConscript
/OP081/cereal/messaging/__init__.py
/OP081/cereal/__init__.py
/OP081/cereal/car.capnp
/OP081/cereal/log.capnp
--------------------------------------------------

/OP081/selfdrive/controls/plannerd.py
  from cereal import car
  from common.params import Params
  from selfdrive.controls.lib.planner import Planner
  from selfdrive.controls.lib.vehicle_model import VehicleModel
  from selfdrive.controls.lib.pathplanner import PathPlanner
  import cereal.messaging as messaging
  def plannerd_thread(sm=None, pm=None):
    CP = car.CarParams.from_bytes(Params().get("CarParams", block=True))
    PL = Planner(CP) # LongitudinalMpc in planner.py
    PP = PathPlanner(CP) # lateral_mpc in pathplanner.py
    VM = VehicleModel(CP)
----- Input: 'model'
      sm = messaging.SubMaster(['carState', 'controlsState', 'radarState', 'model', 'liveParameters'],
                               poll=['radarState', 'model'])
----- Output: 'pathPlan'
      pm = messaging.PubMaster(['plan', 'liveLongitudinalMpc', 'pathPlan', 'liveMpc'])
    sm['liveParameters'].steerRatio = CP.steerRatio
    while True:
      sm.update()
      if sm.updated['model']:
----- Output: PathPlanner(CP).update(CP)
        PP.update(sm, pm, CP, VM)
      if sm.updated['radarState']:
        PL.update(sm, pm, CP, VM, PP)
  def main(sm=None, pm=None):
    plannerd_thread(sm, pm)
  if __name__ == "__main__":
    main()

--- log.CarParams.from_bytes(dat):
    class capnp._StructModule(self, schema, name)
      from_bytes(self, buf, traversal_limit_in_words=None, nesting_limit=None, builder=False)
        from_bytes returns an unpacked object in buf.
        buf (buffer) – Any Python object that supports the buffer interface.
        https://jparyani.github.io/pycapnp/capnp.html

/OP081/selfdrive/controls/lib/pathplanner.py
  from selfdrive.controls.lib.lateral_mpc import libmpc_py
  from selfdrive.controls.lib.drive_helpers import MPC_COST_LAT
  from selfdrive.controls.lib.lane_planner import LanePlanner
  class PathPlanner():
    def __init__(self, CP):
      self.LP = LanePlanner()
      self.steer_rate_cost = CP.steerRateCost
      self.setup_mpc()
    def setup_mpc(self):
      self.libmpc = libmpc_py.libmpc # lateral_mpc
      self.libmpc.init(MPC_COST_LAT.PATH, MPC_COST_LAT.LANE, MPC_COST_LAT.HEADING, self.steer_rate_cost)
      self.mpc_solution = libmpc_py.ffi.new("log_t *")
      self.cur_state = libmpc_py.ffi.new("state_t *")
      self.cur_state[0].x = 0.0
      self.cur_state[0].y = 0.0
      self.cur_state[0].psi = 0.0
      self.cur_state[0].delta = 0.0
      self.angle_steers_des = 0.0
      self.angle_steers_des_mpc = 0.0
      self.angle_steers_des_prev = 0.0
    def update(self, sm, pm, CP, VM):
      v_ego = sm['carState'].vEgo
      angle_steers = sm['carState'].steeringAngle
      angle_offset = sm['liveParameters'].angleOffset
      self.angle_steers_des_prev = self.angle_steers_des_mpc
      curvature_factor = VM.curvature_factor(v_ego)
----- Input: ModelData: LP.parse_model(sm['model'])
      self.LP.parse_model(sm['model'])
      v_ego_mpc = max(v_ego, 5.0)  # avoid mpc roughness due to low speed
----- Run MPC: libmpc_py.libmpc.run_mpc(LP.l_poly)
      self.libmpc.run_mpc(self.cur_state, self.mpc_solution,
                          list(self.LP.l_poly), list(self.LP.r_poly), list(self.LP.d_poly),
                          self.LP.l_prob, self.LP.r_prob, curvature_factor, v_ego_mpc, self.LP.lane_width)
      # reset to current steer angle if not active or overriding
      if active:
        delta_desired = self.mpc_solution[0].delta[1]
        rate_desired = math.degrees(self.mpc_solution[0].rate[0] * VM.sR)
      else:
        delta_desired = math.radians(angle_steers - angle_offset) / VM.sR
        rate_desired = 0.0
      self.angle_steers_des_mpc = float(math.degrees(delta_desired * VM.sR) + angle_offset)
      plan_send = messaging.new_message('pathPlan')
      plan_send.pathPlan.laneWidth = float(self.LP.lane_width)
      plan_send.pathPlan.dPoly = [float(x) for x in self.LP.d_poly]
      plan_send.pathPlan.lPoly = [float(x) for x in self.LP.l_poly]
      plan_send.pathPlan.lProb = float(self.LP.l_prob)
      plan_send.pathPlan.rPoly = [float(x) for x in self.LP.r_poly]
      plan_send.pathPlan.rProb = float(self.LP.r_prob)
----- Run MPC: libmpc_py.libmpc.run_mpc.angle_steers_des_mpc
      plan_send.pathPlan.angleSteers = float(self.angle_steers_des_mpc)
      plan_send.pathPlan.rateSteers = float(rate_desired)
      plan_send.pathPlan.angleOffset = float(sm['liveParameters'].angleOffsetAverage)
----- Output: PathPlanner.update.pm.send."plan_send".messaging.new_message('pathPlan')
      pm.send('pathPlan', plan_send)

/OP081/selfdrive/controls/lib/lane_planner.py
  from common.numpy_fast import interp
  import numpy as np
  from cereal import log
  def model_polyfit(points, path_pinv):
    return np.dot(path_pinv, [float(x) for x in points])
  class LanePlanner:
----- Input: ModelData: LP.parse_model.model_polyfit(,)
    def parse_model(self, md):
      if len(md.leftLane.poly):
        self.l_poly = np.array(md.leftLane.poly)
        self.l_std = float(md.leftLane.std)
        self.r_poly = np.array(md.rightLane.poly)
        self.r_std = float(md.rightLane.std)
        self.p_poly = np.array(md.path.poly)
      else:
        self.l_poly = model_polyfit(md.leftLane.points, self._path_pinv)  # left line
        self.r_poly = model_polyfit(md.rightLane.points, self._path_pinv)  # right line
        self.p_poly = model_polyfit(md.path.points, self._path_pinv)  # predicted path
      self.l_prob = md.leftLane.prob  # left line prob
      self.r_prob = md.rightLane.prob  # right line prob

/OP081/selfdrive/controls/lib/lateral_mpc/libmpc_py.py
  from cffi import FFI
  from common.ffi_wrapper import suffix
  mpc_dir = os.path.dirname(os.path.abspath(__file__))
  libmpc_fn = os.path.join(mpc_dir, "libmpc"+suffix())
  ffi = FFI()
  ffi.cdef("""
  typedef struct {
      double x, y, psi, delta, t;
  } state_t;
  typedef struct {
      double x[21];
      double y[21];
      double psi[21];
      double delta[21];
      double rate[20];
      double cost;
  } log_t;
  void init(double pathCost, double laneCost, double headingCost, double steerRateCost);
  void init_weights(double pathCost, double laneCost, double headingCost, double steerRateCost);
----- Run MPC: libmpc_py.libmpc.run_mpc(LP.l_poly)
  int run_mpc(state_t * x0, log_t * solution,
               double l_poly[4], double r_poly[4], double d_poly[4],
               double l_prob, double r_prob, double curvature_factor, double v_ref, double lane_width);
  """)
  libmpc = ffi.dlopen(libmpc_fn)

--- libmpc_fn == libmpc.so?
    CFFI: an external package providing a C Foreign Function Interface for Python
    https://cffi.readthedocs.io/en/latest/overview.html#overview

/OP081/selfdrive/controls/lib/lateral_mpc/libmpc.so

/OP081/selfdrive/controls/lib/lateral_mpc/lateral_mpc.c
  #include "acado_common.h"
  #include "acado_auxiliary_functions.h"
  typedef struct {
    double x, y, psi, delta, t;
  } state_t;
  typedef struct {
    double x[N+1];
    double y[N+1];
    double psi[N+1];
    double delta[N+1];
    double rate[N];
    double cost;
  } log_t;
----- Run MPC: libmpc_py.libmpc.c.run_mpc.acado_getNWSR()
  int run_mpc(state_t * x0, log_t * solution,
               double l_poly[4], double r_poly[4], double d_poly[4],
               double l_prob, double r_prob, double curvature_factor, double v_ref, double lane_width){
    return acado_getNWSR();}

/OP081/selfdrive/controls/lib/lateral_mpc/SConscript
  Import('env', 'arch')
  cpp_path = [
      "#phonelibs/acado/include",
      "#phonelibs/acado/include/acado",
      "#phonelibs/qpoases/INCLUDE",
      "#phonelibs/qpoases/INCLUDE/EXTRAS",
      "#phonelibs/qpoases/SRC/",
      "#phonelibs/qpoases",
      "lib_mpc_export"]
  mpc_files = [
      "lateral_mpc.c",
      Glob("lib_mpc_export/*.c"),
      Glob("lib_mpc_export/*.cpp"),]

  interface_dir = Dir('lib_mpc_export')
  SConscript(['#phonelibs/qpoases/SConscript'], variant_dir='lib_qp', exports=['interface_dir'])
  env.SharedLibrary('mpc', mpc_files, LIBS=['m', 'qpoases'], LIBPATH=['lib_qp'], CPPPATH=cpp_path)

/OP081/cereal/messaging/__init__.py
  # must be build with scons
  from .messaging_pyx import Context, Poller, SubSocket, PubSocket
  import capnp
  from typing import Optional, List, Union
  from cereal import log
  from cereal.services import service_list
  context = Context()
----- Return: Output: PathPlanner.update.pm.send."dat".messaging.new_message('pathPlan').log.Event.new_message()
  def new_message(service: Optional[str] = None, size: Optional[int] = None) -> capnp.lib.capnp._DynamicStructBuilder:
    dat = log.Event.new_message()
    if service is not None:
      if size is None:
        dat.init(service)
      else:
        dat.init(service, size)
    return dat
  class SubMaster():
    def __init__(self, services: List[str], , , ):
      self.sock = {}
      self.data = {}
      for s in services:
          self.sock[s] = sub_sock(s, poller=p, addr=addr, conflate=True)
        try:
          data = new_message(s)
        self.data[s] = getattr(data, s)
  class PubMaster():
    def __init__(self, services: List[str]):
      self.sock = {}
      for s in services:
        self.sock[s] = pub_sock(s)
    def send(self, s: str, dat: Union[bytes, capnp.lib.capnp._DynamicStructBuilder]) -> None:
      if not isinstance(dat, bytes):
        dat = dat.to_bytes()
      self.sock[s].send(dat)

----- Return: Output: PathPlanner.update.pm.send."dat".messaging.new_message('pathPlan').log.capnp.new_message()
--- log.Event.new_message() => capnp.new_message(): Initialize a New Cap’n Proto Object
      addresses = addressbook_capnp.AddressBook.new_message()
    https://capnproto.github.io/pycapnp/quickstart.html
    https://capnproto.org/language.html
--- dat.to_bytes() => capnp.to_bytes(): https://jparyani.github.io/pycapnp/quickstart.html

/OP081/cereal/__init__.py
  import capnp
  log = capnp.load(os.path.join(CEREAL_PATH, "log.capnp"))
  car = capnp.load(os.path.join(CEREAL_PATH, "car.capnp"))

--- https://capnp.readthedocs.io/en/latest/quickstart.html
    load a Capnproto Schema: log.capnp
    read Schema Language: https://capnproto.org/language.html

/OP081/cereal/car.capnp
  using Cxx = import "./include/c++.capnp";
  $Cxx.namespace("cereal");
  struct CarEvent @0x9b1657f34caf3ad3 {
  struct CarState {
    events @13 :List(CarEvent);
    # car speed
    vEgo @1 :Float32;         # best estimate of speed
    aEgo @16 :Float32;        # best estimate of acceleration
    vEgoRaw @17 :Float32;     # unfiltered speed from CAN sensors
    yawRate @22 :Float32;     # best estimate of yaw rate
    standstill @18 :Bool;
    wheelSpeeds @2 :WheelSpeeds;
    # gas pedal, 0.0-1.0
    gas @3 :Float32;        # this is user + computer
    gasPressed @4 :Bool;    # this is user pedal only
    steeringAngle @7 :Float32;       # deg
    steeringRate @15 :Float32;       # deg/s
    steeringTorque @8 :Float32;      # TODO: standardize units
    steeringTorqueEps @27 :Float32;  # TODO: standardize units
    struct WheelSpeeds {
      # optional wheel speeds
      fl @0 :Float32;
      fr @1 :Float32;
      rl @2 :Float32;
      rr @3 :Float32;
  struct RadarData @0x888ad6581cf0aacb {
  struct CarControl {
  struct CarParams {
    carName @0 :Text;
    carFingerprint @1 :Text;
    enableCamera @4 :Bool;
    minEnableSpeed @7 :Float32;
    minSteerSpeed @8 :Float32;
    steerMaxBP @11 :List(Float32);
    steerMaxV @12 :List(Float32);
    # things about the car in the manual
    mass @17 :Float32;             # [kg] running weight
    wheelbase @18 :Float32;        # [m] distance from rear to front axle
    centerToFront @19 :Float32;   # [m] GC distance to front axle
    steerRatio @20 :Float32;       # [] ratio between front wheels and steering wheel angles
    steerRatioRear @21 :Float32;  # [] rear steering ratio wrt front steering (usually 0)
    # things we can derive
    rotationalInertia @22 :Float32;    # [kg*m2] body rotational inertia
    tireStiffnessFront @23 :Float32;   # [N/rad] front tire coeff of stiff
    tireStiffnessRear @24 :Float32;    # [N/rad] rear tire coeff of stiff
    lateralParams @48 :LateralParams;
    lateralTuning :union {
      pid @26 :LateralPIDTuning;
      indi @27 :LateralINDITuning;
      lqr @40 :LateralLQRTuning;}
    vEgoStopping @29 :Float32; # Speed at which the car goes into stopping state
    directAccelControl @30 :Bool; # Does the car have direct accel control or just gas/brake
    stoppingControl @31 :Bool; # Does the car allows full control even at lows speeds when stopping
    startAccel @32 :Float32; # Required acceleraton to overcome creep braking
    steerRateCost @33 :Float32; # Lateral MPC cost on steering rate
    steerControlType @34 :SteerControlType;
    stoppingBrakeRate @52 :Float32; # brake_travel/s while trying to stop
    startingBrakeRate @53 :Float32; # brake_travel/s while releasing on restart
    steerActuatorDelay @36 :Float32; # Steering wheel actuator delay in seconds
    openpilotLongitudinalControl @37 :Bool; # is openpilot doing the longitudinal control?
    carVin @38 :Text; # VIN number queried during fingerprinting
    fingerprintSource @49: FingerprintSource;
    struct LateralParams {
      torqueBP @0 :List(Int32);
      torqueV @1 :List(Int32);}
    struct LateralPIDTuning {
    struct LongitudinalPIDTuning {
    struct LateralINDITuning {
      outerLoopGain @0 :Float32;
      innerLoopGain @1 :Float32;
      timeConstant @2 :Float32;
      actuatorEffectiveness @3 :Float32;}
    struct LateralLQRTuning {
    enum SafetyModel {
      silent @0;
      hondaNidec @1;
      toyota @2;
    enum SteerControlType {
      torque @0;
      angle @1;}
    struct CarFw {
      ecu @0 :Ecu;
      fwVersion @1 :Data;
      address @2: UInt32;
      subAddress @3: UInt8;}
    enum Ecu {
      eps @0;
      esp @1;
      fwdRadar @2;
      fwdCamera @3;
      engine @4;
      unknown @5;
      transmission @8; # Transmission Control Module
      srs @9; # airbag
      gateway @10; # can gateway
      hud @11; # heads up display
      combinationMeter @12; # instrument cluster
      # Toyota only
      dsu @6;
      apgs @7;}
    enum FingerprintSource {
      can @0;
      fw @1;
      fixed @2; }  }

/OP081/cereal/log.capnp
  using Cxx = import "./include/c++.capnp";
  $Cxx.namespace("cereal");
  using Car = import "car.capnp";
  struct LiveParametersData {
    angleOffset @2 :Float32;
    angleOffsetAverage @3 :Float32;
    steerRatio @5 :Float32;
    yawRate @7 :Float32;
    posenetSpeed @8 :Float32;
  struct Event {
      model @9 :ModelData;
----- Return: Output: PathPlanner.update.pm.send."PathPlan".messaging.new_message('pathPlan').log.capnp.new_message
      pathPlan @64 :PathPlan;
      liveParameters @61 :LiveParametersData;
      carState @22 :Car.CarState;
