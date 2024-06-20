--------------------------------------------------
CCCCCCCCCCCCCCCCCCCCCCCC Code Snippets
'''
Extra Code and Notes from zcCS1-6
'''
CAN 10------------------------------------------------------

--- Key Words: net_outputs, kj::ArrayPtr, pm.send, modelV2,
               CC.actuators, CI.apply(CC), can_sends, kj::Array,
--- 2021.4.27 below from commaai github

openpilot/selfdrive/modeld/modeld.cc
  void run_model(ModelState &model, VisionIpcClient &vipc_client) {
    PubMaster pm({"modelV2", "cameraOdometry"}); // messaging
    SubMaster sm({"lateralPlan", "roadCameraState"});
    ModelDataRaw model_buf = model_eval_frame(&model, buf->buf_cl, buf->width, buf->height,
----- Output:
    model_publish(pm, ..., model_buf, ...,
                  kj::ArrayPtr<const float>(model.output.data(), model.output.size()));
----- main:
  int main(int argc, char **argv) {
    ModelState model;
    model_init(&model, device_id, context);
    VisionIpcClient vipc_client = VisionIpcClient
    if (vipc_client.connected) {
      run_model(model, vipc_client);
----- JLL 2021.6.7.
pthread_create() called in modeld.cc is declared in <pthread.h> that is included in
  comma2/openpilot0.8.1/installer/installer.c and
  https://github.com/commaai/openpilot/tree/master/installer
  but not in my Git clone OP081/installer
POSIX thread (pthread) libraries are a standards based thread API for C/C++.
  It allows one to spawn a new concurrent process flow. It is most effective
  on multi-processor or multi-core systems where the process flow can be
  scheduled to run on another processor thus gaining speed through parallel or distributed processing.
The Portable Operating System Interface (POSIX) is a family of standards
  specified by the IEEE Computer Society for maintaining compatibility between operating systems.
https://www.geeksforgeeks.org/multithreading-c-2/
----- JLL

openpilot/selfdrive/modeld/models/driving.cc
  void model_publish(PubMaster &pm, ..., const ModelDataRaw &net_outputs, ...,
                     kj::ArrayPtr<const float> raw_pred)
    MessageBuilder msg;
    auto framed = msg.initEvent().initModelV2();
    if (send_raw_pred) {
      framed.setRawPredictions(raw_pred.asBytes());
    fill_model(framed, net_outputs);
    pm.send("modelV2", msg);
--- std::malloc() allocates memory without initializing.
    calloc() allocates memory with initializing to 0. malloc is faster than calloc.
    https://www.geeksforgeeks.org/difference-between-malloc-and-calloc-with-examples/

openpilot/selfdrive/controls/controlsd.py
  import math
    self.sm = messaging.SubMaster(['modelV2', 'lateralPlan',
      meta = self.sm['modelV2'].meta
      # Curvature & Steering angle
      steer_angle_without_offset = math.radians(CS.steeringAngleDeg - params.angleOffsetAverageDeg)
      angle_steers_des += params.angleOffsetDeg
      controlsState.steeringAngleDesiredDeg = angle_steers_des
      elif self.CP.lateralTuning.which() == 'indi':
        controlsState.lateralControlState.indiState = lac_log

cereal/messaging/__init__.py
--- for python code
  import capnp
  def new_message(service:
  def pub_sock(endpoint: str) -> PubSocket:
  def sub_sock(endpoint: str, poller: Optional[Poller] = None, addr: str = "127.0.0.1",
  def update_msgs(self, cur_time: float, msgs: List[capnp.lib.capnp._DynamicStructReader]) -> None:
  class SubMaster():
  class PubMaster():

CAN 10------------------------------------------------------

CAN 18------------------------------------------------------
/home/jinn/OP081/selfdrive/boardd/panda.cc
--- Mutex or Mutual Exclusion Object is used to give access to a resource to only one process at a time.
    Mutex lock for Linux Thread Synchronization
--- Nibble: A Nibble consits of 4 bits. A nibble containing all 0s will be
      the iteger 0, while a nibble containing all 1s will be the integer 15 (very useful for hex numbers)
--- C++ Vertical Line | is bitwise OR. For example:
    (1 | 2) == (01 | 10) == 11 == 3;
    (5 | 3) == (101 | 011) == 111 == 4+2+1 == 7

/home/jinn/OP081/selfdrive/launcher.py
  import cereal.messaging as messaging
      messaging.context = messaging.Context()

/home/jinn/OP081/cereal/__init__.py
  import capnp
  log = capnp.load(os.path.join(CEREAL_PATH, "log.capnp"))

/home/jinn/OP081/cereal/gen/cpp/car.capnp.h
  namespace capnp {
  namespace schemas {
  CAPNP_DECLARE_SCHEMA(9b1657f34caf3ad3);
  CAPNP_DECLARE_SCHEMA(baa8c5d505f727de);
  enum class EventName_baa8c5d505f727de: uint16_t {
    friend class ::capnp::MessageBuilder;
--- C++ friend Class: https://www.programiz.com/cpp-programming/friend-function-class
    Scope resolution operator in C++ ::capnp::
    https://www.geeksforgeeks.org/scope-resolution-operator-in-c/

--- 2021.5.21: commaai/openpilot/selfdrive/boardd/boardd.cc
  #include "cereal/messaging/messaging.h"
  #include "selfdrive/boardd/panda.h"
--- boardd210516.cc
    kj::Array<capnp::word> can_data;
    panda->can_receive(can_data);
    auto bytes = can_data.asBytes();
    pm.send("can", bytes.begin(), bytes.size());

--- 2021.5.22: SConstruct is not the first running boardd.cc. It is manager.py.
/home/jinn/OP081/SConstruct
env = Environment(
  CCFLAGS=[
  CPPPATH=cpppath + [
    "#selfdrive/boardd",
    "#cereal/messaging",
    "#cereal",
    "#opendbc/can",
  CC='clang',
  CXX='clang++',
  LIBPATH=libpath + [
    "#cereal",
    "#phonelibs",
    "#opendbc/can",
    "#selfdrive/boardd",
    "#selfdrive/common",
SConscript(['selfdrive/boardd/SConscript'])

--- Boot order of comma 2: C2BootOrder.txt
    https://drive.google.com/drive/folders/1hAVTbdAzmHV9qgLMREr9Pqw5s8hHkvNn
---------- Boot order of comma 2 (or PC) in Linux ---------- JLL 2020.7.24

The boot sequence defines which devices a computer should check for the operating system's boot files.

A step by step tutorial for understanding Linux boot sequence
https://www.slashroot.in/linux-booting-process-step-step-tutorial-understanding-linux-boot-sequence
Step 1: Power Supply & SMPS (Switching Mode Power Supply).
Step 2: Bootstrapping. ...
Step 3: The Role of BIOS in booting process. ...
Step 4: MBR and GRUB. ...
Step 5: Loading The kernel Image.

BIOS (basic input/output system) is the program a personal computer's microprocessor
  uses to get the computer system started after you turn it on.
It also manages data flow between the computer's operating system and attached
  devices such as the hard disk, video adapter, keyboard, mouse and printer.

An introduction to the Linux boot and startup processes
Step 1: BIOS POST
Step 2: Boot loader (GRUB2)
Step 3: Kernel initialization
Step 4: Start systemd, the mother of all processes.
Systemd provides a standard process for controlling what programs run when a Linux system boots up.

--- Check systemd
icms@jinnliu:/lib/systemd/system$ ls

When we install a new package, during the installation, its unit configuration
  file is also installed/generated in the /usr/lib/systemd/system directory.

--- SCons: OPNotesCode1.txt
    https://drive.google.com/file/d/1VznxlVXSCHjoE2KAUZfuYJfpnoNQQjuG/view?usp=sharing

---------- Step : Code Structure Running controlsd.py
/home/jinn/openpilot/selfdrive/controls/controlsd.py
  from cereal import car, log
  import cereal.messaging as messaging
  from selfdrive.car.car_helpers import get_car, get_startup_event, get_one_can
  from selfdrive.controls.lib.events import Events, ET
  class Controls:
    def __init__(self, sm=None, pm=None, can_sock=None):
      self.pm = messaging.PubMaster(['sendcan', 'controlsState', 'carState',
                                     'carControl', 'carEvents', 'carParams'])
      self.sm = messaging.SubMaster(['thermal', 'health', 'frame', 'model', 'liveCalibration',
                                     'dMonitoringState', 'plan', 'pathPlan', 'liveLocationKalman'])
      self.can_sock = can_sock
      self.can_sock = messaging.sub_sock('can', timeout=can_timeout)
      get_one_can(self.can_sock)
      self.CI, self.CP = get_car(self.can_sock, self.pm.sock['sendcan'], has_relay)
      car_recognized = self.CP.carName != 'mock'
      controller_available = self.CP.enableCamera and self.CI.CC is not None and not passive
      self.CC = car.CarControl.new_message()
      self.startup_event = get_startup_event(car_recognized, controller_available)
      # controlsd is driven by can recv, expected at 100Hz
      self.rk = Ratekeeper(100, print_delay_threshold=None)
    def data_sample(self):
      """Receive data from sockets and update carState"""
      # Update carState from CAN
      can_strs = messaging.drain_sock_raw(self.can_sock, wait_for_one=True)
----- Output
      CS = self.CI.update(self.CC, can_strs)
      self.sm.update(0)
      # When the panda and controlsd do not agree on controls_allowed
      # we want to disengage openpilot. However the status from the panda goes through
      return CS
    def publish_logs(self, CS, start_time, actuators, v_acc, a_acc, lac_log):
      """Send actuators and hud commands to the car, send controlsstate and MPC logging"""
      # carState
      car_events = self.events.to_msg()
      cs_send = messaging.new_message('carState')
      cs_send.valid = CS.canValid
      cs_send.carState = CS
      cs_send.carState.events = car_events
----- Output
      self.pm.send('carState', cs_send)
      # carEvents - logged every second or on change
      if (self.sm.frame % int(1. / DT_CTRL) == 0) or (self.events.names != self.events_prev):
        ce_send = messaging.new_message('carEvents', len(self.events))
        ce_send.carEvents = car_events
        self.pm.send('carEvents', ce_send)
    def step(self):
      start_time = sec_since_boot()
      # Sample data from sockets and get a carState
      CS = self.data_sample()
      self.update_events(CS)
      # Publish data
----- Output
      self.publish_logs(CS, start_time, actuators, v_acc, a_acc, lac_log)
    def controlsd_thread(self):
      while True:
      self.step()
  def main(sm=None, pm=None, logcan=None):
    controls = Controls(sm, pm, logcan)
    controls.controlsd_thread()
  if __name__ == "__main__":
    main()

/home/jinn/openpilot/selfdrive/car/car_helpers.py
  from selfdrive.car.fingerprints import eliminate_incompatible_cars, all_known_cars
  from selfdrive.car.fw_versions import get_fw_versions, match_fw_to_car
  from selfdrive.car import gen_empty_fingerprint
  import cereal.messaging as messaging
  from cereal import car, log
  def get_one_can(logcan):
    while True:
      can = messaging.recv_one_retry(logcan)
      if len(can.can) > 0:
        return can
  def load_interfaces(brand_names):
    for brand_name in brand_names:
      path = ('selfdrive.car.%s' % brand_name)
      CarInterface = __import__(path + '.interface', fromlist=['CarInterface']).CarInterface
      if os.path.exists(BASEDIR + '/' + path.replace('.', '/') + '/carstate.py'):
        CarState = __import__(path + '.carstate', fromlist=['CarState']).CarState
      if os.path.exists(BASEDIR + '/' + path.replace('.', '/') + '/carcontroller.py'):
        CarController = __import__(path + '.carcontroller', fromlist=['CarController']).CarController
      for model_name in brand_names[brand_name]:
        ret[model_name] = (CarInterface, CarController, CarState)
    return ret
  def _get_interface_names():
    # read all the folders in selfdrive/car and return a dict where:
    for car_folder in [x[0] for x in os.walk(BASEDIR + '/selfdrive/car')]:
        brand_name = car_folder.split('/')[-1]
        model_names = __import__('selfdrive.car.%s.values' % brand_name, fromlist=['CAR']).CAR
        model_names = [getattr(model_names, c) for c in model_names.__dict__.keys() if not c.startswith("__")]
        brand_names[brand_name] = model_names
    return brand_names
  # imports from directory selfdrive/car/<name>/
  interface_names = _get_interface_names()
  interfaces = load_interfaces(interface_names)
  def only_toyota_left(candidate_cars):
    return all(("TOYOTA" in c or "LEXUS" in c) for c in candidate_cars) and len(candidate_cars) > 0
  def fingerprint(logcan, sendcan, has_relay):
    finger = gen_empty_fingerprint()
    while not done:
      a = get_one_can(logcan)
      for can in a.can:
        # need to independently try to fingerprint both bus 0 and 1 to work
        if can.src in range(0, 4):
          finger[can.src][can.address] = len(can.dat)
        for b in candidate_cars:
          # Toyota needs higher time to fingerprint, since DSU does not broadcast immediately
        if only_toyota_left(candidate_cars[b]):
          frame_fingerprint = 100  # 1s
        if len(candidate_cars[b]) == 1:
          if frame > frame_fingerprint:
            # fingerprint done
            car_fingerprint = candidate_cars[b][0]
      car_fingerprint = list(fw_candidates)[0]   # python list() Constructor
      source = car.CarParams.FingerprintSource.fw
    return car_fingerprint, finger, vin, car_fw, source
  def get_car(logcan, sendcan, has_relay=False):
    candidate, fingerprints, vin, car_fw, source = fingerprint(logcan, sendcan, has_relay)
    CarInterface, CarController, CarState = interfaces[candidate]
    car_params = CarInterface.get_params(candidate, fingerprints, has_relay, car_fw)
    car_params.fingerprintSource = source
    return CarInterface(car_params, CarController, CarState), car_params

/home/jinn/openpilot/selfdrive/car/__init__.py
  def gen_empty_fingerprint():
    return {i: {} for i in range(0, 4)}
  def dbc_dict(pt_dbc, radar_dbc, chassis_dbc=None, body_dbc=None):
    return {'pt': pt_dbc, 'radar': radar_dbc, 'chassis': chassis_dbc, 'body': body_dbc}
  def apply_toyota_steer_torque_limits(apply_torque, apply_torque_last, motor_torque, LIMITS):
  def crc8_pedal(data):
  def create_gas_command(packer, gas_amount, idx):
  def make_can_msg(addr, dat, bus):
    return [addr, 0, dat, bus]

/home/jinn/openpilot/opendbc/can/parser_pyx.pyx
  from common cimport CANParser as cpp_CANParser
  from common cimport SignalParseOptions, MessageParseOptions, dbc_lookup, SignalValue, DBC
  cdef class CANParser:
    cdef:
      cpp_CANParser *can
      const DBC *dbc
      map[string, uint32_t] msg_name_to_address
      map[uint32_t, string] address_to_msg_name
      vector[SignalValue] can_values
    def __init__(self, dbc_name, signals, checks=None, bus=0):
      self.can = new cpp_CANParser(bus, dbc_name, message_options_v, signal_options_v)

/home/jinn/openpilot/opendbc/can/parser.cc
  CANParser::CANParser(int abus, const std::string& dbc_name,
          const std::vector<MessageParseOptions> &options,
          const std::vector<SignalParseOptions> &sigoptions)
    : bus(abus) {
    dbc = dbc_lookup(dbc_name);
      MessageState state = {
        .address = op.address,
      const Msg* msg = NULL;
      for (int i=0; i<dbc->num_msgs; i++) {
        if (dbc->msgs[i].address == op.address) {
          msg = &dbc->msgs[i];
      state.size = msg->size;
        const Signal *sig = &msg->sigs[i];
          state.parse_sigs.push_back(*sig);
          state.vals.push_back(0);
            state.vals.push_back(sigop.default_value);
      message_states[state.address] = state;
  void CANParser::UpdateCans(uint64_t sec, const capnp::List<cereal::CanData>::Reader& cans) {
      DEBUG("got %d messages\n", msg_count);
      // parse the messages
        auto cmsg = cans[i];
        auto state_it = message_states.find(cmsg.getAddress());
        memcpy(dat, cmsg.getDat().begin(), cmsg.getDat().size());

---------- Step : Code Structure Running dbc.py
/home/jinn/openpilot/opendbc/can/process_dbc.py
  from opendbc.can.dbc import dbc
  def process(in_fn, out_fn):
    dbc_name = os.path.split(out_fn)[-1].replace('.cc', '')
    can_dbc = dbc(in_fn)
    msgs = [(address, msg_name, msg_size, sorted(msg_sigs, key=lambda s: s.name not in ("COUNTER", "CHECKSUM")))
          for address, ((msg_name, msg_size), msg_sigs) in sorted(can_dbc.msgs.items()) if msg_sigs]
    elif can_dbc.name.startswith(("toyota_", "lexus_")):
      checksum_type = "toyota"
      checksum_size = 8
      counter_size = None
      checksum_start_bit = 7
      counter_start_bit = None
      little_endian = False
    for address, msg_name, _, sigs in msgs:
      dbc_msg_name = dbc_name + " " + msg_name
    parser_code = template.render(dbc=can_dbc, checksum_type=checksum_type, msgs=msgs, def_vals=def_vals, len=len)
    with open(out_fn, "w") as out_f:
      out_f.write(parser_code)
  def main():
      print("usage: %s dbc_directory output_filename" % (sys.argv[0],))
    dbc_dir = sys.argv[1]
    out_fn = sys.argv[2]
    dbc_name = os.path.split(out_fn)[-1].replace('.cc', '')
    in_fn = os.path.join(dbc_dir, dbc_name + '.dbc')
    process(in_fn, out_fn)
  if __name__ == '__main__':
    main()

openpilot/release/files_common
  opendbc/can/process_dbc.py

openpilot/release/build_devel.sh
  cp -pR --parents $(cat release/files_common)

/home/jinn/openpilot/opendbc/can/dbc.py
  DBCSignal = namedtuple(
    "DBCSignal", ["name", "start_bit", "size", "is_little_endian", "is_signed",
                "factor", "offset", "tmin", "tmax", "units"])
  class dbc():
    def __init__(self, fn):
      self.name, _ = os.path.splitext(os.path.basename(fn))
      with open(fn, encoding="ascii") as f:
      self.txt = f.readlines()
      bo_regexp = re.compile(r"^BO\_ (\w+) (\w+) *: (\w+) (\w+)")
      sg_regexp = re.compile(r"^SG\_ (\w+) : (\d+)\|(\d+)@(\d+)([\+|\-]) \(([0-9.+\-eE]+),([0-9.+\-eE]+)\) \[([0-9.+\-eE]+)\|([0-9.+\-eE]+)\] \"(.*)\" (.*)")
      sgm_regexp = re.compile(r"^SG\_ (\w+) (\w+) *: (\d+)\|(\d+)@(\d+)([\+|\-]) \(([0-9.+\-eE]+),([0-9.+\-eE]+)\) \[([0-9.+\-eE]+)\|([0-9.+\-eE]+)\] \"(.*)\" (.*)")
      val_regexp = re.compile(r"VAL\_ (\w+) (\w+) (\s*[-+]?[0-9]+\s+\".+?\"[^;]*)")
        if l.startswith("BO_ "):
          dat = bo_regexp.match(l)
        name = dat.group(2)
        size = int(dat.group(3))
        ids = int(dat.group(1), 0)  # could be hex
        self.msgs[ids] = ((name, size), [])
        self.msgs[ids][1].append(
        if l.startswith("SG_ "):
          DBCSignal(sgname, start_bit, signal_size, is_little_endian,
                    is_signed, factor, offset, tmin, tmax, units))
        if l.startswith("VAL_ "):
    def reverse_bytes(self, x):
      return ((x & 0xff00000000000000) >> 56) | \
    def encode(self, msg_id, dd):
    def decode(self, x, arr=None, debug=False):
      msg = self.msgs.get(x[0])
      name = msg[0][0]
      if debug:
----- Output:
        print(name)
        st = x[2].ljust(8, b'\x00')
        start_bit = s[1]
        signal_size = s[2]
        little_endian = s[3]
        signed = s[4]
        factor = s[5]
        offset = s[6]
        if little_endian:
            le = struct.unpack("<Q", st)[0]
          tmp = le
          shift_amount = start_bit
        else:
            be = struct.unpack(">Q", st)[0]
          tmp = be
          b1 = (start_bit // 8) * 8 + (-start_bit - 1) % 8
          shift_amount = 64 - (b1 + signal_size)
        tmp = tmp * factor + offset
          out[arr.index(s[0])] = tmp
      return name, out

openpilot/selfdrive/car/toyota/carstate.py
  from opendbc.can.can_define import CANDefine
  from selfdrive.car.interfaces import CarStateBase
  from opendbc.can.parser import CANParser
  from selfdrive.car.toyota.values import CAR, DBC, STEER_THRESHOLD, TSS2_CAR, NO_STOP_TIMER_CAR
  class CarState(CarStateBase):
      ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
      ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
      if self.CP.carFingerprint == CAR.PRIUS:
      return ret
    @staticmethod
    def get_can_parser(CP):
      signals = [
        ("STEER_ANGLE", "STEER_ANGLE_SENSOR", 0),
      return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

/home/jinn/openpilot/selfdrive/car/toyota/interface.py
  from selfdrive.car.toyota.values import Ecu, ECU_FINGERPRINT, CAR, TSS2_CAR, FINGERPRINTS
  from selfdrive.car.interfaces import CarInterfaceBase
  EventName = car.CarEvent.EventName
  class CarInterface(CarInterfaceBase):
    @staticmethod
    def get_params(candidate, fingerprint=fingerprint(), has_relay=False, car_fw=[]):  # pylint: disable=dangerous-default-value
      ret = CarInterfaceBase.get_std_params(candidate, fingerprint, has_relay)
      ret.carName = "toyota"
      if candidate == CAR.PRIUS:
        stop_and_go = True
      ret.enableCamera = is_ecu_disconnected(fingerprint[0], FINGERPRINTS, ECU_FINGERPRINT, candidate, Ecu.fwdCamera) or has_relay
      smartDsu = 0x2FF in fingerprint[0]
      ret.enableDsu = is_ecu_disconnected(fingerprint[0], FINGERPRINTS, ECU_FINGERPRINT, candidate, Ecu.dsu) and candidate not in TSS2_CAR
      ret.enableGasInterceptor = 0x201 in fingerprint[0]
      ret.openpilotLongitudinalControl = ret.enableCamera and (smartDsu or ret.enableDsu or candidate in TSS2_CAR)
      ret.minEnableSpeed = -1. if (stop_and_go or ret.enableGasInterceptor) else 19. * CV.MPH_TO_MS
      ret.communityFeature = ret.enableGasInterceptor or ret.enableDsu or smartDsu
      ret.longitudinalTuning.deadzoneBP = [0., 9.]
      return ret
    # returns a car.CarState
    def update(self, c, can_strings):
      # ******************* do can recv *******************
      self.cp.update_strings(can_strings)
      self.cp_cam.update_strings(can_strings)
      ret = self.CS.update(self.cp, self.cp_cam)
      events = self.create_common_events(ret)
      ret.events = events.to_msg()
      self.CS.out = ret.as_reader()
      return self.CS.out

openpilot/selfdrive/debug/dump.py
  parser.add_argument('--addr', default='127.0.0.1')
  if args.addr != "127.0.0.1":
    os.environ["ZMQ"] = "1"
    messaging.context = messaging.Context()

cereal/messaging/socketmaster.cc
#include "messaging.h"
SubMaster::SubMaster(const std::initializer_list<const char *> &service_list, const char *address,
    SubSocket *socket = SubSocket::create(ctx.ctx_, name, address ? address : "127.0.0.1", true);
    SubMessage *m = new SubMessage{
      .name = name,
      .socket = socket,
      .freq = serv->frequency,
      .ignore_alive = inList(ignore_alive, name),
      .allocated_msg_reader = malloc(sizeof(capnp::FlatArrayMessageReader))};
    messages_[socket] = m;
    services_[name] = m;

cereal/SConscript
Import('env', 'envCython', 'arch', 'QCOM_REPLAY')
import shutil
# Build cereal
schema_files = ['log.capnp', 'car.capnp', 'legacy.capnp']
env.Command(["gen/c/include/c++.capnp.h", "gen/c/include/java.capnp.h"], [], "mkdir -p " + gen_dir.path + "/c/include && touch $TARGETS")
env.Command([f'gen/cpp/{s}.c++' for s in schema_files] + [f'gen/cpp/{s}.h' for s in schema_files],
            schema_files,
            f"capnpc --src-prefix={cereal_dir.path} $SOURCES -o c++:{gen_dir.path}/cpp/")
# Build messaging
messaging_objects = env.SharedObject([
  'messaging/messaging.cc',
  'messaging/impl_zmq.cc',
  'messaging/impl_msgq.cc',
  'messaging/msgq.cc',
  'messaging/socketmaster.cc',
])
envCython.Program('messaging/messaging_pyx.so', 'messaging/messaging_pyx.pyx', LIBS=envCython["LIBS"]+[messaging_lib, "zmq"])

openpilot/selfdrive/debug/can_printer.py
import cereal.messaging as messaging
def can_printer(bus=0, max_msg=None, addr="127.0.0.1"):
  logcan = messaging.sub_sock('can', addr=addr)
  canbus = int(os.getenv("CAN", bus))
    can_recv = messaging.drain_sock(logcan, wait_for_one=True)
          msgs[y.address].append(y.dat)

/home/jinn/openpilot/panda/__init__.py
  from .python import Panda, PandaDFU, BASEDIR, build_st

/home/jinn/openpilot/panda/python/__init__.py
  import socket
  from .dfu import PandaDFU  # pylint: disable=import-error
  from .isotp import isotp_send, isotp_recv  # pylint: disable=import-error
  BASEDIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../")
  def build_st(target, mkfile="Makefile", clean=True):
  def parse_can_buffer(dat):
  class WifiHandle(object):
    def __init__(self, ip="192.168.0.10", port=1337):
      self.sock = socket.create_connection((ip, port))
    def __recv(self):
      ret = self.sock.recv(0x44)
      length = struct.unpack("I", ret[0:4])[0]
      return ret[4:4 + length]
  class Panda(object):
    # matches cereal.car.CarParams.SafetyModel
    SAFETY_TOYOTA = 2
    def __init__(self, serial=None, claim=True):
      self.connect(claim)
    def connect(self, claim=True, wait=False):
      if self._serial == "WIFI":
          for device in context.getDeviceList(skip_on_error=True):
            if device.getVendorID() == 0xbbaa and device.getProductID() in [0xddcc, 0xddee]:
                this_serial = device.getSerialNumber()
                self._serial = this_serial
                print("opening device", self._serial, hex(device.getProductID()))
    def set_can_forwarding(self, from_bus, to_bus):
    def set_can_enable(self, bus_num, enable):
    def set_can_speed_kbps(self, bus, speed):
    def can_send_many(self, arr, timeout=CAN_SEND_TIMEOUT_MS):
      snds = []
      transmit = 1
      extended = 4
      for addr, _, dat, bus in arr:
        assert len(dat) <= 8
        if DEBUG:
          print(f"  W 0x{addr:x}: 0x{dat.hex()}")
        if addr >= 0x800:
          rir = (addr << 3) | transmit | extended
        else:
          rir = (addr << 21) | transmit
        snd = struct.pack("II", rir, len(dat) | (bus << 4)) + dat
        snd = snd.ljust(0x10, b'\x00')
        snds.append(snd)
            for s in snds:
              self._handle.bulkWrite(3, s)
    def can_send(self, addr, dat, bus, timeout=CAN_SEND_TIMEOUT_MS):
      self.can_send_many([[addr, None, dat, bus]], timeout=timeout)
    def can_recv(self):
      dat = bytearray()
        dat = self._handle.bulkRead(1, 0x10 * 256)
      return parse_can_buffer(dat)
CAN 18------------------------------------------------------
