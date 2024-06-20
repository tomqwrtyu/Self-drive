CB 5  Jinn-Liang Liu  2023.5.22 - 7.19  All Rights Reserved. © Copyright 2023

========== OP Coding Notes: Run, Change, Read ==========

"openpilot (OP)", "dragonpilot (DP)", "legacypilot (LP)",
"Comma Two (C2A/C2B)", and "Comma Body (CB)" Notes

Abbreviations: CAN, Ctr, Lcz, Md, Net, Nav, UI
  AR (Autonomous Rolling), CAN (Control Area Network), CBQ (CB Question), Ctr (Control),
  Lcz (Localization), Md (Model), Net (Network), Nav (Navigation), UI (User Interface), XJ (Changed by JLL),
  PID (Proportional-Integral-Derivative Control in [Liu23a]),
  MPC (Model Predictive Control in [Liu23b]),
  KF  (Kalman Filter Control in [Liu23c]),

========== Summary of Coding Notes ==========

230714: OK Msg 1: Pipe 1a 1b: PC: scons "/cerealJLL", Run "controlsdJLL.py", "controlsdJLL1.py"
--- Run "controlsdJLL.py" for AM 1
--- Run "controlsd091JLL.py" on "COMMA BODY"
--- Read "Cap’n Proto Schema Language, /cereal, car.capnp, log.capnp"
--- Msg 1: "cereal/messaging" + "capnp" > create "/cerealJLL"
--- Pipe 1a: c++.capnp > carJLL.capnp > logJLL.capnp > servicesJLL.py > messagingJLL/__init__.py >
    SConstruct > SConscript > c++.capnp.h > genJLL/c, genJLL/cpp ...
--- Pipe 1b: genJLL/... > servicesJLL.h > messagingJLL/messaging.cc ... > messagingJLL/messaging_pyx.pyx >
    messagingJLL/messaging_pyx.so
--- scons "/cerealJLL"
--- Run "controlsdJLL1.py" import carJLL

230708: OK KF 1-2: Pipe 1 3: PC: Run 'kalman1.py', 'kalman2.py', 'kalman3.py'
--- KF 1: [Liu23c] + kalman1.py
--- Pipe 1: Kalman: kalman1.py > kalman2.py > kalman3.py > test_simple_kalmanJLL.py >
      test_simple_kalman.py, test_kinematic_kf.py, kalman_laika.py (Kalman.ipynb, GNSS)
      kinematic_kf.py, ekf_sym.py, car_kf.py, gnss_kf.py, live_kf.py, loc_kf.py, lane_kf.py
--- Pipe 2: locationd.cc > pm({"liveLocationKalman"}) > 230619 Pipe 2d > controlsd.py > plannerd.py
    or > navd.py
--- Run Read 'kalman1.py'
--- OK  Output: /openpilot/aKalman/fig1KF1.png
--- KF 2: [Liu23c] + 'kalman2a.py' + 'kalman2.py' + 'kalman3.py' + 'radardJLL.py'
--- Run 'kalman2a.py'
--- OK: KF1D converged very well!
--- Run 'kalman3.py'
--- OK  Output: /openpilot/aKalman/fig1KF2.png
--- Pipe 3: radard.py > RI = RadarInterface(CP) > RD = RadarD() > kalman_params =
--- Pipe 3a: radard.py > radar_helpers.py > simple_kalman.py > simple_kalman_impl.pyx > simple_kalman_impl.cpp
--- Pipe 3b: KalmanParams > RI.update() > RD.update(rr) > Track() > KF1D > Cluster() > RD.tracks > 'liveTracks'
--- Read [Liu23c] for KalmanParams > KF1D([[v_lead], [0.0]], K_A, K_C, K_K)
--- ToDo 1: Run "/aKalman/test_kinematic_kfJLL.py", "car_kf.py", "test_compare.py",
--- ToDo 2: Read /car/toyota/carstate.py > update_speed_kf > self.v_ego_kf

230702: OK Run PC: Pipe 1b 3b: Run Read 'run_paramsd_on_routeJLL.py' for 'paramsd.py'
--- Pipe 1a: rlog.bz2 > 'liveLocationKalman' > ParamsLearner() > learner.kf.x > plt.figure()
--- Pipe 1b: rlog.bz2 > Route(,) > load_segment() > LogReader() > msgs > CP = m.carParams >
--- #--- CP = ( carName = "toyota", carFingerprint = "TOYOTA PRIUS 2017", ...,
    'liveLocationKalman' > learner.kf.x > ao_avg = math.degrees(x[States.ANGLE_OFFSET] > results > plt.figure()
--- Pipe 1c: run_paramsd_on_routeJLL.py > paramsd.py > car_kf.py > kalmanfilter.py
--- Pipe 2: rlog.bz2 > 'liveParameters' > results_log > plt.figure()
--- Pipe 3a: paramsd.py > sm.updated['liveLocationKalman'] > x = learner.kf.x > angle_offset_average = clip() >
    liveParameters.angleOffsetAverageDeg = angle_offset_average > pm.send('liveParameters',)
--- Q 1: Why is figure().Kalman C so jagged? A: No Pipe 3a.clip() in Pipe 1b.
--- Pipe 3b: paramsd.py > main() > PubMaster(['liveParameters']) > params_reader = Params()
    > params['steerRatio'] > learner = ParamsLearner(, params['steerRatio'],,))
    > sm.updated['liveLocationKalman'] > x = learner.kf.x
    > liveParameters.steerRatio = float(x[States.STEER_RATIO]) > pm.send('liveParameters',)
--- Pipe 3c: paramsd.py < params.py < params_pyx.pyx < params_pyx.cpp < params.cc < util.cc
--- Pipe 3d: paramsd.py < car_kf.py < kalmanfilter.py
--- Q 2: Why Kalman A != liveParameters A (steerRatio)? A:
--- ToDo 1: Read 'test_vehicle_modelJLL.py'
--- ToDo 2: for "rlog.bz2" in /data/media/0/dashcam/
--- ToDo 3: for "from_bytes" CP = car.CarParams.from_bytes(params_reader.get("CarParams", block=True))

230619: OK MPC 2: Pipe 1f: PC: Read 'lat_mpc.py','lateral_planner.py' Run 'test_lateral_mpcJLL.py
--- MPC 2: [Liu23b] + 'lat_mpc.py'
--- Pipe 1a: plannerd.py < lateral_planner.py < lat_mpc.py < acados_ocp_solver_pyx.pyx < acados_solver.pxd
    < acados_solver.in.pxd < acados_solver.in.c < acados_ocp_solver_pyx.c < acados_solver_lat.c
--- Pipe 1b: solver = AcadosOcpSolverCython < cdef class AcadosOcpSolverCython
    < * cdef class AcadosOcpSolverCython: #
--- Pipe 1c: solver.solve() < cimport acados_solver, cdef class AcadosOcpSolverCython: def solve(self):
    < acados_solver.acados_solve(self.capsule)
--- Pipe 1d: acados_solver < int acados_solve "lat_acados_solve"(nlp_solver_capsule * capsule)
    < int acados_solve "{{ model.name }}_acados_solve"(nlp_solver_capsule * capsule)
    < int {{ model.name }}_acados_solve({{ model.name }}_solver_capsule* capsule)
      {int solver_status = ocp_nlp_solve(); return solver_status;}
    < ACADOS_SYMBOL_EXPORT int ocp_nlp_solve();
--- Pipe 1e: acados_solve() < ACADOS_SYMBOL_EXPORT int lat_acados_solve()
    < lat_acados_solve(){int solver_status = ocp_nlp_solve(); return solver_status }
--- Pipe 1f: lateral_planner.py > LateralPlanner > self.lat_mpc.reset(x0=self.x0) >
    def update() > measured_curvature = sm['controlsState'].curvature > md = sm['modelV2'] >
    self.path_xyz = ... md.position.x > self.plan_yaw = np.array(md.orientation.z) >
    self.lat_mpc.run(self.x0, p, y_pts, heading_pts, yaw_rate_pts) >
    self.x0[3] = measured_curvature * self.v_ego > def publish() >
    lateralPlan.psis = self.lat_mpc.x_sol > (..lat_mpc.x_sol[0:CONTROL_N, 3]..) >
    lateralPlan.curvatureRates = [..lat_mpc.u_sol[0:CONTROL_N - 1]] > pm.send('lateralPlan', )
--- Read controlsd.py > "measured_curvature" > lateral_planner.py
--- Pipe 2a: self.VM.update_params(x, sr) < self.VM = VehicleModel() < controlsd.py < VehicleModel
    < vehicle_model.py < ret.tireStiffnessFront, = scale_tire_stiffness() < /toyota/interface.py
    < def scale_tire_stiffness() < /car/__init__.py
--- Pipe 2b: kalmanfilter.py > KalmanFilter > car_kf.py > CarKalman(KalmanFilter) >
    paramsd.py > ParamsLearner > self.kf = CarKalman() > def main() > sm = messaging.SubMaster(..'liveLocationKalman') >
      learner = ParamsLearner() > x = learner.kf.x > liveParameters.steerRatio = float(x[States.STEER_RATIO]) >
      pm.send('liveParameters', )
--- Pipe 2c: controlsd.py > def state_control() > lp = self.sm['liveParameters'] >
    x = max(lp.stiffnessFactor,) > sr = max(lp.steerRatio,) > self.VM.update_params(x, sr) >
    def publish_logs() > steer_angle_without_offset = math.radians(..lp.angleOffsetDeg) >
    curvature = -self.VM.calc_curvature(steer_angle_without_offset,,) >
    controlsState.curvature = curvature > self.pm.send('controlsState',) >
    lateral_planner.py > measured_curvature = sm['controlsState'].curvature
--- Pipe 2d: 'liveLocationKalman' > KalmanFilter > CarKalman(KalmanFilter) > ParamsLearner >
    'liveParameters' > LateralPlanner
--- ToDo 1: Run selfdrive/modeld/tests/'test_modeld.py', cereal/messaging/tests/'test_pub_sub_master.py'

230530: OK MPC 1: PC: Run Read "mpc.py"
--- MPC 1: [Liu23b] + "mpc.py" A. Sakai
--- Pipe: CubicSpline1D() > CubicSpline2D() > get_switch_back_course() >
    do_simulation() > RefTrajectory() > NewtonIter() > LinearMPC()

230529: NG. PID 1: AR 3: C2+body: Change "controlsd.py", "carcontroller.py"
--- Do Ctr 1a: Yaw < 0 => (0, 0, 0) => (X=62.6, Y=-7.7, Z=0). Want: (X=0, Y=0, Z=0)
--- PID 1: [Liu23a] + Apdx 1-3 CB5.py
--- ToDo 1: Do PID 1
--- ToDo 2: Ctr 1a w/ PID 1 is harder than I thought. It is suitable for student's project.
--- AR 3: CB moves from Point A to B with PID 1 in (X, Y) system.

230525: OK AR 2a: XJ 6: Ctr 3: Nav 1: AMN 1: C2+body: Change "controlsd.py", "carcontroller.py"
--- Nav 1: /selfdrive/assets/navigation/screenshot.png, GPSNMEAData,
--- XJ 6a: Change controlsd.py: Delete 230521 Pipe 3b, 3c: Delete these:
    self.LoC = LongControl(self.CP)
--- Run web.py
--- XJ 6b: Do AR 2a, CBQ 2, 230521 ToDo 2 using "distance_traveled":
      speed_diff_desired = -0.013  # XJ 6b for AR 2
    if self.distance_traveled < 3.0 and self.odometer < 4.0:  # AR 2: forward
--- I add self.odometer.
--- Ctr 3: CB rolled grossly from (0,0) to (3.24, -0.57) m. =>
--- ToDo: AMN 1: AR 2 + Md 1 + Nav 1 (PID + Path + GPS)

===== Details of Coding Notes =====

230714: OK Msg 1: Pipe 1a 1b: PC: scons "/cerealJLL", Run "controlsdJLL.py", "controlsdJLL1.py"
--- Run "controlsdJLL.py" for AM 1
    #--- vars(self.sm) = {'frame': -1, 'updated': {'testJoystick': False}, 'rcv_time': {'testJoystick': 0.0}, ..., 'recv_dts': {'testJoystick': deque([0.0, ..., 0.0], maxlen=100)},
      'sock': {'testJoystick': <cereal.messaging.messaging_pyx.SubSocket object at 0x7fef68f70090>}, 'freq': {'testJoystick': 0.0},
      'data': {'testJoystick': <log.capnp:Joystick builder ()>},
      'valid': {'testJoystick': True}, 'logMonoTime': {'testJoystick': 0},
      'poller': <cereal.messaging.messaging_pyx.Poller object at 0x7fef68fe5fa0>,
      'non_polled_services': [], 'ignore_average_freq': ['testJoystick'],
      'ignore_alive': ['testJoystick']}
    #--- actuators = ( gas = 0, brake = 0, steer = 0, steeringAngleDeg = 45, accel = -0.3,
    #---  longControlState = off, speed = 0, curvature = 0, steerOutputCan = 0 )
    Polling, or interrogation, refers to actively sampling the status of an external device
      by a client program as a synchronous activity.
--- Run "controlsd091JLL.py" with "body"
    #--- vars(self.events) = {'events': [66, 34, 98], 'static_events': [66, 34], 'events_prev': {74: 0, 34: 1, 98: 0, ...}}
    CarInterface, CarController, CarState = interfaces["mock"]  # OK
    CarInterface, CarController, CarState = interfaces["body"]  # NG KeyError: 'body'
--- KeyError: 'body' Why?
      /controlsd091JLL.py
      from selfdrive.car.car_helpers import interfaces
        /car_helpers.py
        from selfdrive.car.interfaces import get_interface_attr
        interface_names = _get_interface_names()
          def _get_interface_names() -> Dict[str, List[str]]:
            for brand_name, model_names in get_interface_attr("CAR").items():
              /interfaces.py
              def get_interface_attr(attr: str, combine_brands: bool = False, ignore_none: bool = False) -> Dict[str, Any]:
        interfaces = load_interfaces(interface_names)
          #--- interfaces = {'COMMA BODY': (<class 'selfdrive.car.body.interface.CarInterface'>, <class 'selfdrive.
          def load_interfaces(brand_names):
              for model_name in brand_names[brand_name]:
                ret[model_name] = (CarInterface, CarController, CarState)
            return ret
--- Change to
    CarInterface, CarController, CarState = interfaces["COMMA BODY"]
--- OK
    #--- vars(self.CI) = {
      'CP': <car.capnp:CarParams builder (carName = "body", ...)>,
      'VM': <...>,
        'frame': 0, 'steering_unpressed': 0, 'low_speed_alert': False, 'no_steer_warning': False,
        'silent_steer_warning': True, 'v_ego_cluster_seen': False,
      'CS': <...>,
        'can_parsers': [<...>, , , , ],
        'cp': <...>, 'cp_cam': None, 'cp_adas': None, 'cp_body': None, 'cp_loopback': None,
      'CC': <...> }
    #--- self.CP = (
      carName = "body", carFingerprint = "COMMA BODY", enableGasInterceptor = false,
      pcmCruise = true, enableCameraDEPRECATED = false, enableDsu = false, enableApgsDEPRECATED = false,
      minEnableSpeed = -1, minSteerSpeed = -inf, safetyModelDEPRECATED = silent, safetyParamDEPRECATED = 0,
      mass = 9, wheelbase = 0.406, centerToFront = 0.17864, steerRatio = 0.5, steerRatioRear = 0,
      rotationalInertia = 0.34798431, tireStiffnessFront = 1104.0082, tireStiffnessRear = 1371.238,
      longitudinalTuning = ( kpBP = [0], kpV = [1], kiBP = [0], kiV = [1], deadzoneBP = [0],
        deadzoneV = [0], kf = 1 ), lateralTuning = (), steerLimitAlert = false, vEgoStopping = 0.5,
      directAccelControlDEPRECATED = false, stoppingControl = true, startAccel = 0, steerRateCostDEPRECATED = 0,
      steerControlType = angle, radarUnavailable = true, steerActuatorDelay = 0, openpilotLongitudinalControl = true,
      isPandaBlackDEPRECATED = false, dashcamOnly = false, safetyModelPassiveDEPRECATED = silent,
      transmissionType = unknown, radarTimeStep = 0.05, communityFeatureDEPRECATED = false,
      steerLimitTimer = 1, fingerprintSource = can, networkLocation = fwdCamera, minSpeedCanDEPRECATED = 0,
      stoppingDecelRate = 0.8, startingAccelRateDEPRECATED = 0, maxSteeringAngleDegDEPRECATED = 0,
      fuzzyFingerprint = false, enableBsm = false, hasStockCameraDEPRECATED = false,
      longitudinalActuatorDelayUpperBound = 0.15, vEgoStarting = 0.5, stopAccel = -2,
      longitudinalActuatorDelayLowerBound = 0.15,
      safetyConfigs = [ ( safetyModel = body, safetyParamDEPRECATED = 0, safetyParam2DEPRECATED = 0,
          safetyParam = 0 ) ], wheelSpeedFactor = 0.008587, flags = 0, alternativeExperience = 0,
      notCar = true, maxLateralAccel = inf, autoResumeSng = true, startingState = false,
      experimentalLongitudinalAvailable = false )
    #--- self.CS_prev = (
      vEgo = 0, wheelSpeeds = (fl = 0, fr = 0, rl = 0, rr = 0), gas = 0, gasPressed = false,
      brake = 0, brakePressed = false, steeringAngleDeg = 0, steeringTorque = 0, steeringPressed = false,
      cruiseState = ( enabled = true, speed = 0, available = true, speedOffset = 0, standstill = false,
                      nonAdaptive = false, speedCluster = 0 ),
      gearShifter = drive, steeringRateDeg = 0, aEgo = 0, vEgoRaw = 0, standstill = false,
      brakeLightsDEPRECATED = false, leftBlinker = false, rightBlinker = false,
      yawRate = 0, genericToggle = false, doorOpen = false, seatbeltUnlatched = false,
      canValid = false, steeringTorqueEps = 0, clutchPressed = false, steeringRateLimitedDEPRECATED = false,
      stockAeb = false, stockFcw = false, espDisabled = false, leftBlindspot = false,
      rightBlindspot = false, steerFaultTemporary = false, steerFaultPermanent = false,
      steeringAngleOffsetDeg = 0, brakeHoldActive = false, parkingBrake = false,
      canTimeout = false, fuelGauge = 0, accFaulted = false, charging = false, vEgoCluster = 0,
      regenBraking = false )
    #--- self.CC = (
      enabled = false, gasDEPRECATED = 0, brakeDEPRECATED = 0, steeringTorqueDEPRECATED = 0,
      cruiseControl = ( cancel = true, resume = false, speedOverrideDEPRECATED = 0,
                        accelOverrideDEPRECATED = 0, override = false ),
      hudControl = ( speedVisible = false, setSpeed = 70.833336, lanesVisible = false,
                     leadVisible = false, visualAlert = none, audibleAlert = none,
                     rightLaneVisible = true, leftLaneVisible = true, rightLaneDepart = false,
                     leftLaneDepart = false ),
      actuators = ( gas = 0, brake = 0, steer = 0, steeringAngleDeg = 0, accel = 0,
                    longControlState = off, speed = 0, curvature = 0, steerOutputCan = 0 ),
      activeDEPRECATED = false, rollDEPRECATED = 0, pitchDEPRECATED = 0,
      latActive = false, longActive = false, leftBlinker = false, rightBlinker = false )
--- Read "Cap’n Proto Schema Language, /cereal, car.capnp, log.capnp"
    Cap’n Proto Schema Language
      /car.capnp
      using Cxx = import "./include/c++.capnp";
        Cxx is the scope (an alias, the namespace) of c++.capnp.
        /c++.capnp
          $namespace("capnp::annotations");
            Annotation can 1) control details of a particular code generator,
                           2) assist dynamic message processing like “hide from the public”
          annotation namespace(file): Text;
          annotation name(field, enumerant, struct, enum, interface, method, param, group, union): Text;
      $Cxx.namespace("cereal");  # Cxx creates the namespace "cereal" of car.capnp
      @0x8e2af1e708af8b8d
        unique file ID, generated by `capnp id` and put it in schema
        Unique IDs are manually or automatically assigned to files and compound types
        IDs provide a relatively short yet unambiguous way to refer to a type or
          annotation from another context.
        Annotations in the compiled schema are identified only by their ID
      /log.capnp
      struct Map(Key, Value) {
        struct Entry {  # OK: Map(Key, Value).Entry, NG: Map.Entry(Key, Value)
--- Msg 1: "cereal/messaging" + "capnp" > create "/cerealJLL"
--- Pipe 1a: c++.capnp > carJLL.capnp > logJLL.capnp > servicesJLL.py > messagingJLL/__init__.py >
    SConstruct > SConscript > c++.capnp.h > genJLL/c, genJLL/cpp ...
--- scons "/cerealJLL"
  (sconsvenv) jinn@Liu:~/openpilot/cerealJLL$ scons
--- Error
    scons: *** No tool module 'cython' found in /home/jinn/sconsvenv/lib/python3.8/site-packages/SCons/Tool
    File "/home/jinn/openpilot/cerealJLL/SConstruct", line 36, in <module>
--- Add to /cerealJLL/SConstruct
    py_include = sysconfig.get_paths()['include']
    envCython["CPPPATH"] += [py_include, np.get_include()]
--- Error
--- rename /openpilot/SConstruct to SConstructLP091 /SConstructJLL to SConstruct
--- Change /openpilot/SConstruct
    SConscript(['cerealJLL/SConscript'])
--- Error
    NameError: name 'gen_dir' is not defined:
     File "/home/jinn/openpilot/cerealJLL/SConscript", line 12:
    cerealJLL/logJLL.capnp:1:1: error: File does not declare an ID.
    I've generated one for you. Add this line to your file: @0xb659a66238cbe898;
    cerealJLL/logJLL.capnp:15: error: Union must have at least two members.
    cerealJLL/carJLL.capnp:1:1: error: File does not declare an ID.
    I've generated one for you.  Add this line to your file: @0xb4b6aeca6a643c1c;
--- OK: scons: done building targets. > genJLL/... /c++.capnp.h, carJLL.capnp.c++ ...
--- Copy messaging/messaging.cc, .h, impl_zmq.cc ...
    (11 files) to messagingJLL/messaging.cc ...
--- Change to "cerealJLL/messagingJLL/" in all files
--- Pipe 1b: genJLL/... > servicesJLL.h > messagingJLL/messaging.cc ... > messagingJLL/messaging_pyx.pyx >
    messagingJLL/messaging_pyx.so
--- scons "/cerealJLL"
--- Error
    ./cerealJLL/messagingJLL/messaging.h:77:84: error: use of undeclared identifier 'cereal'
    cerealJLL/messagingJLL/socketmaster.cc:57:3: error: use of undeclared identifier 'cereal'
--- Change to cerealJLL::
--- OK: scons: done building targets.
--- Run "controlsdJLL1.py" import carJLL
  (sconsvenv) jinn@Liu:~/openpilot$ python controlsdJLL1.py
--- Error
    File "capnp/lib/capnp.pyx", line 4342, in capnp.lib.capnp.load
    File "capnp/lib/capnp.pyx", line 3553, in capnp.lib.capnp.SchemaParser.load
    OSError: File not found: /home/jinn/openpilot/cerealJLL/log.capnp
    File "/home/jinn/openpilot/cerealJLL/messagingJLL/__init__.py", line 10, in <module>
      from common.realtime import sec_since_boot
    File "/home/jinn/openpilot/system/hardware/base.py", line 5, in <module>
      from cereal import log
    File "capnp/lib/capnp.pyx", line 4342, in capnp.lib.capnp.load
    File "capnp/lib/capnp.pyx", line 3572, in capnp.lib.capnp.SchemaParser.load
    capnp.lib.capnp.KjException: home/jinn/openpilot/cereal/include/c++.capnp:0:
    failed: Duplicate ID @0xbdf87d7bb8304e81.
--- Change messagingJLL/__init__.py to
      #from common.realtime import sec_since_boot
    import time
    sec_since_boot = time.time
--- Run "controlsdJLL1.py"
    #--- vars(self.sm) = {
      'data': {'testJoystick': <logJLL.capnp:Joystick builder ()>},
      'valid': {'testJoystick': True}, 'logMonoTime': {'testJoystick': 0}}
--- OK: same as above but
--- Error: TypeError: 'SubMaster' object is not subscriptable
--- Add to messagingJLL/__init__.py
    def __getitem__(self, s: str) -> capnp.lib.capnp._DynamicStructReader:
--- OK. Done.
--- ToDo 1: Determine MIN_LATERAL_CONTROL_SPEED for "body"
--- ToDo 2: Run "/controls/tests/test_startup.py"
--- ToDo 3: Run "/selfdrive/test/longitudinal_maneuvers/plant.py"
--- ToDo 4: Run "/car/tests/test_models.py"
--- ToDo 5: Run "/can/tests/test_parser.py"

230708: OK KF 1-2: Pipe 1 3: PC: Run 'kalman1.py', 'kalman2.py', 'kalman3.py'
--- KF 1: [Liu23c] + kalman1.py
--- Pipe 1: Kalman: kalman1.py > kalman2.py > kalman3.py > test_simple_kalmanJLL.py >
      test_simple_kalman.py, test_kinematic_kf.py, kalman_laika.py (Kalman.ipynb, GNSS)
      kinematic_kf.py, ekf_sym.py, car_kf.py, gnss_kf.py, live_kf.py, loc_kf.py, lane_kf.py
--- Pipe 2: locationd.cc > pm({"liveLocationKalman"}) > 230619 Pipe 2c > controlsd.py > plannerd.py
    or > navd.py
--- Run Read 'kalman1.py'
--- OK  Output: /openpilot/aKalman/fig1KF1.png
--- KF 2: [Liu23c] + 'kalman2a.py' + 'kalman2.py' + 'kalman3.py' + 'radardJLL.py'
--- Run 'kalman2a.py'
--- OK: KF1D converged very well!
    #--- Case 1   see 'kalman2a.py'
    #--- i, v_meas, x, z = 0   10.0 [1.23, 2.97]  1.2
    #--- i, v_meas, x, z = 1   10.0 [2.34, 5.57]  2.28 ...
    #--- i, v_meas, x, z = 198 10.0 [10.01, 0.07] 10.01
    #--- i, v_meas, x, z = 199 10.0 [10.01, 0.06] 10.01 => lead car: constant speed, zero accel
    #--- Case 2
    #--- i, v_meas, x, z = 0   10.0 [1.99, 2.86]  1.85
    #--- i, v_meas, x, z = 1   10.0 [3.72, 5.14]  3.47
    #--- i, v_meas, x, z = 77  10.0 [9.99, -0.01] 9.99
    #--- i, v_meas, x, z = 78  10.0 [10.0, -0.01] 10.0
    #--- i, v_meas, x, z = 198 10.0 [10.0, 0.0]   10.0
    #--- i, v_meas, x, z = 199 10.0 [10.0, 0.0]   10.0
--- Run 'kalman2.py'
--- OK
    #--- vars(kalman_params) = {'A': [[1.0, 0.05], [0.0, 1.0]], 'C': [1.0, 0.0], 'K': [[0.19887], [0.28555]]}
    #--- vars(tracks[0]) = {'cnt': 0, 'aLeadTau': 1.5, 'K_A': [[1.0, 0.05], [0.0, 1.0]], 'K_C': [1.0, 0.0], 'K_K': [[0.19887], [0.28555]], 'kf': <common.kalman.simple_kalman_impl.KF1D object at 0x7eff3efe08b0>}
    #--- v_lead = -0.2
--- Run 'kalman3.py'
--- OK  Output: /openpilot/aKalman/fig1KF2.png
    #--- RVfile[1] = /home/jinn/dataB/UHD--2018-05-01--08-13-53--29/processed_log/CAN/radar/value
    #--- i, j, radar_tms[idx+j], can_rvls[j][0], can_rvls[j][5] = 0 0 1780.836002299 64.8875 1072.0
    #--- i, j, radar_tms[idx+j], can_rvls[j][0], can_rvls[j][5] = 1199 0 1840.795340755 258.6375 1089.0
    #--- len(tracks) = 16
--- Run 'radardJL.py'
--- OK
--- Run 'test_simple_kalmanJLL.py'
--- OK
    #--- kf_speed, kf_old_speed = 0.001450074021704495 0.028155962005257607
--- Read 'kalman2a.py' + 'kalman2.py' + 'kalman3.py' + 'radardJLL.py', Write KF 2 [Liu23c]
--- Pipe 3: radard.py > RI = RadarInterface(CP) > RD = RadarD() > kalman_params =
    KalmanParams(radar_ts) > can_strings > radar raw = rr = RI.update(can_strings) >
    RD.update(rr) >
      pt in rr.points > ar_pts[pt.trackId] = [pt.dRel, pt.yRel, pt.vRel, pt.measured] >
      rpt = ar_pts[ids] > v_lead = rpt[2] + v_ego_hist[0] >
      tracks[ids] = Track(v_lead, kalman_params) >
        radar_helpers.py > Track() >
          kf = KF1D([[v_lead], [0.0]], K_A, K_C, K_K) >
            simple_kalman.py > import KF1D > simple_kalman_impl.pyx > KF1D >
            simple_kalman_impl.pxd > simple_kalman_impl.cpp >
              KF1D: # <<<<<<<<<<<<<<, update(self, meas): # <<<<<<<<<<<<<<
          update() > kf.update(vLead) > kf.x > vLeadK, aLeadK, aLeadTau
      self.tracks[ids].update(rpt[0], rpt[1], rpt[2], v_lead, rpt[3]) >
      track_pts = [self.tracks[iden].get_key_for_cluster() for iden in idens] >
      Cluster() > clusters > get_lead(clusters) > leadOne, leadTwo
    tracks = RD.tracks > 'liveTracks' >
    dat.liveTracks[cnt]{ "trackId":, "dRel":, "yRel":, "vRel":, } >
    pm.send('liveTracks', dat)
--- Pipe 3a: radard.py > radar_helpers.py > simple_kalman.py > simple_kalman_impl.pyx > simple_kalman_impl.cpp
--- Pipe 3b: KalmanParams > RI.update() > RD.update(rr) > Track() > KF1D > Cluster() > RD.tracks > 'liveTracks'
--- Read [Liu23c] for KalmanParams > KF1D([[v_lead], [0.0]], K_A, K_C, K_K)
--- ToDo 1: Run "/aKalman/test_kinematic_kfJLL.py", "car_kf.py", "test_compare.py",
--- ToDo 2: Read /car/toyota/carstate.py > update_speed_kf > self.v_ego_kf

230702: OK Run PC: Pipe 1b 3b: Run Read 'run_paramsd_on_routeJLL.py' for 'paramsd.py'
--- 230619 Pipe 2d: 'liveLocationKalman' > CarKalman(KalmanFilter) > ParamsLearner > 'liveParameters'
--- Run 'test_paramsdJLL.py'
--- Error
    TypeError: __init__() missing 1 required positional argument: 'generated_dir'
--- Change to selfdrive/debug/internal/run_paramsd_on_route.py
  (sconsvenv) jinn@Liu:~/openpilot$ python test_route_libraryJLL.py
--- Error
    File "run_paramsd_on_routeJLL.py", line 38, in <module>
      route = Route(ROUTE)
    File "/home/jinn/openpilot/tools/lib/api.py", line 17, in request
      raise UnauthorizedError('Unauthorized. Authenticate with tools/lib/auth.py')
--- Change to /openpilot/tools/lib/tests/test_route_library.py
  (sconsvenv) jinn@Liu:~/openpilot$ python test_route_libraryJLL.py
    #--- s = 4cf7a6ad03080c90|2021-09-29--13-46-36--1
    #--- s.data_dir = /data/media/0/realdata
--- OK
--- Change: route = Route(ROUTE, "/home/jinn/dataC")
--- Run 'run_paramsd_on_routeJLL.py'
  (sconsvenv) jinn@Liu:~/openpilot$ python run_paramsd_on_routeJLL.py
    Loading /home/jinn/dataC/8bfda98c9c9e4291|2020-05-11--03-00-57--61/rlog.bz2
    {'carFingerprint': 'TOYOTA PRIUS 2017', 'steerRatio': 16.47,
     'stiffnessFactor': 1.0, 'angleOffsetAverageDeg': 0.349}
--- OK
--- Read 'run_paramsd_on_routeJLL.py'
--- Pipe 1a: rlog.bz2 > 'liveLocationKalman' > ParamsLearner() > learner.kf.x > plt.figure()
--- Pipe 1b: rlog.bz2 > Route(,) > load_segment() > LogReader() > msgs > CP = m.carParams >
--- #--- CP = ( carName = "toyota", carFingerprint = "TOYOTA PRIUS 2017", ...,
                mass = 1517.1876, wheelbase = 2.7, centerToFront = 1.188, steerRatio = 15.74, ...
    < params = {'steerRatio': CP.steerRatio, ...} < learner = ParamsLearner()
      < paramsd.py < ParamsLearner < self.kf = CarKalman()
        < car_kf.py < CarKalman(KalmanFilter) < kalmanfilter.py < KalmanFilter < def x(self)
    'liveLocationKalman' > learner.kf.x > ao_avg = math.degrees(x[States.ANGLE_OFFSET] > results > plt.figure()
--- Pipe 1c: run_paramsd_on_routeJLL.py < paramsd.py < car_kf.py < kalmanfilter.py
--- Pipe 2: rlog.bz2 > 'liveParameters' > results_log > plt.figure()
--- Pipe 3a: paramsd.py > sm.updated['liveLocationKalman'] > x = learner.kf.x > angle_offset_average = clip() >
    liveParameters.angleOffsetAverageDeg = angle_offset_average > pm.send('liveParameters',)
--- Q 1: Why is figure().Kalman C so jagged? A: No Pipe 3a.clip() in Pipe 1b.
--- Pipe 3b: paramsd.py > main() > PubMaster(['liveParameters']) > params_reader = Params()
      < params.py < import Params < params_pyx.pyx < cdef cppclass c_Params "Params":
        < params_pyx.cpp < cdef class Params: < params.cc < Params::Params(const std::string &path)
          < Params::get(const std::string &key, bool block) > util::read_file(getParamPath(key))
            < util.cc > std::string read_file() < std::ifstream f() < f.read()
    > params = params_reader.get("LiveParameters") > params = json.loads(params)
    > params['steerRatio'] > learner = ParamsLearner(, params['steerRatio'],,))
      < paramsd.py < ParamsLearner < self.kf = CarKalman()
        < car_kf.py < CarKalman(KalmanFilter) < kalmanfilter.py < KalmanFilter < def x(self)
    > sm.updated['liveLocationKalman'] > x = learner.kf.x
    > liveParameters.steerRatio = float(x[States.STEER_RATIO]) > pm.send('liveParameters',)
--- Pipe 3c: paramsd.py < params.py < params_pyx.pyx < params_pyx.cpp < params.cc < util.cc
--- Pipe 3d: paramsd.py < car_kf.py < kalmanfilter.py
--- Q 2: Why Kalman A != liveParameters A (steerRatio)? A:
--- Run 'test_vehicle_modelJLL.py'
  (sconsvenv) jinn@Liu:~/openpilot$ python test_vehicle_modelJLL.py
--- OK
--- ToDo 1: Read 'test_vehicle_modelJLL.py'
--- ToDo 2: for "rlog.bz2" in /data/media/0/dashcam/
    selfdrive/loggerd/main.cc
      selfdrive/loggerd/uploader.py
      selfdrive/athena/athenad.py
      selfdrive/loggerd/tests/test_loggerd.py
      selfdrive/loggerd/tests/test_loggerd.cc
    system/loggerd/loggerd.cc
      system/loggerd/uploader.py
      system/loggerd/tests/test_loggerd.py
--- ToDo 3: for "from_bytes" CP = car.CarParams.from_bytes(params_reader.get("CarParams", block=True))
    1. capnp Example in https://jparyani.github.io/pycapnp/capnp.html
    2. cereal/messaging/demo.py
    3. std::string read_file(const std::string& fn)

230619: OK MPC 2: Pipe 1f: PC: Read 'lat_mpc.py','lateral_planner.py' Run 'test_lateral_mpcJLL.py
--- MPC 2: [Liu23b] + 'lat_mpc.py'
--- Run 'test_lateral_mpcJLL.py'
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/Control$ python test_lateral_mpcJLL.py
--- OK:
    x_sol, u_sol = run_mpc()  # JLL
      #--- x_sol = [[0.00000000e+00 0.00000000e+00 0.00000000e+00 0.00000000e+00] [2.92968750e-01 0.00000000e+00 0.00000000e+00 0.00000000e+00]
      #--- x_sol.shape, u_sol.shape = (33, 4) (32, 1)
--- Read 'lat_mpc.py'
    /selfdrive/controls/lib/lateral_mpc_lib/SConscript
      source_list = ['lat_mpc.py', f'{acados_dir}/larch64/lib/libacados.so',
      f'{acados_templates_dir}/acados_solver.in.c',
      acados_ocp_solver_pyx = File("#third_party/acados/acados_template/acados_ocp_solver_pyx.pyx")
      libacados_ocp_solver_pxd = File(f'{gen}/acados_solver.pxd')
    /selfdrive/controls/plannerd.py
      def main(sm=None, pm=None):
        plannerd_thread(sm, pm)
          def plannerd_thread(sm=None, pm=None):
            CP = car.CarParams.from_bytes(params.get("CarParams", block=True))
            longitudinal_planner = LongitudinalPlanner(CP)
            |lateral_planner = LateralPlanner(CP)
              selfdrive/controls/lib/lateral_planner.py
              class LateralPlanner:
                def __init__(self, CP):
                  |self.lat_mpc = LateralMpc()
                    /selfdrive/controls/lib/lateral_mpc_lib/lat_mpc.py
                      from selfdrive.controls.lib.lateral_mpc_lib.c_generated_code.acados_ocp_solver_pyx import AcadosOcpSolverCython
--- acados_ocp_solver_pyx defined by SConscript
                      class LateralMpc():
                        def __init__(self, x0=np.zeros(X_DIM)):
                          |self.solver = AcadosOcpSolverCython(MODEL_NAME, ACADOS_SOLVER_TYPE, N)
                            /third_party/acados/acados_template/acados_ocp_solver_pyx.pyx
                              |cimport acados_solver
                                /selfdrive/controls/lib/lateral_mpc_lib/c_generated_code/acados_solver.pxd
                                  int acados_solve "lat_acados_solve"(nlp_solver_capsule * capsule)
--- Q: Is 'int acados_solve "f"()' a Cython or acados syntax? A: acados as defined in acados_solver.in.pxd
                                    /third_party/acados/acados_template/c_templates_tera/acados_solver.in.pxd
                                      int acados_solve "{{ model.name }}_acados_solve"(nlp_solver_capsule * capsule)
                                        /third_party/acados/acados_template/c_templates_tera/acados_solver.in.c
                                          int {{ model.name }}_acados_solve({{ model.name }}_solver_capsule* capsule)
                                            {// solve NLP
                                            int solver_status = ocp_nlp_solve(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);
                                              /third_party/acados/include/acados_c/ocp_nlp_interface.h
                                                ACADOS_SYMBOL_EXPORT int ocp_nlp_solve(ocp_nlp_solver *solver, ocp_nlp_in *nlp_in, ocp_nlp_out *nlp_out);
--- ocp_nlp_solve() defined and compiled in f'{acados_dir}/larch64/lib/libacados.so'
                                            return solver_status;}
                              |cdef class AcadosOcpSolverCython:
                                  /selfdrive/controls/lib/lateral_mpc_lib/c_generated_code/acados_ocp_solver_pyx.c
                                  /* "acados_template/acados_ocp_solver_pyx.pyx":52
                                  * cdef class AcadosOcpSolverCython:  # <<<<<<<<<<<<<<
                                  struct __pyx_obj_15acados_template_21acados_ocp_solver_pyx_AcadosOcpSolverCython {
                                cdef acados_solver.nlp_solver_capsule *capsule
                                cdef str model_name
                                cdef int N
                                cdef str nlp_solver_type
                                def __cinit__(self, model_name, nlp_solver_type, N):
                                def solve(self):"""  Solve the ocp with current input.  """
                                  return acados_solver.acados_solve(self.capsule)
                                    /selfdrive/controls/lib/lateral_mpc_lib/c_generated_code/acados_ocp_solver_pyx.c
                                      #include "acados_solver_lat.h"
                                      /* "acados_template/acados_ocp_solver_pyx.pyx":112
                                      *   Solve the ocp with current input.
                                      *   return acados_solver.acados_solve(self.capsule)  # <<<<<<<<<<<<<<
                                      __pyx_t_1 = __Pyx_PyInt_From_int(lat_acados_solve(__pyx_v_self->capsule)); if (unlikely(!__pyx_t_1)) __PYX_ERR(0, 112, __pyx_L1_error)
                                        /selfdrive/controls/lib/lateral_mpc_lib/c_generated_code/acados_solver_lat.h
                                          ACADOS_SYMBOL_EXPORT int lat_acados_solve(lat_solver_capsule * capsule);
                                        /selfdrive/controls/lib/lateral_mpc_lib/c_generated_code/acados_solver_lat.c
                                          int lat_acados_solve(lat_solver_capsule* capsule)
                                          {// solve NLP
                                            int solver_status = ocp_nlp_solve(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);
                                            return solver_status;}
                                def set(self, int stage, str field_, value_):
                                def cost_set(self, int stage, str field_, value_):
                                def constraints_set(self, int stage, str field_, value_):
                          |self.reset(x0)
                            def reset(self, x0=np.zeros(X_DIM)):
                              self.x_sol = np.zeros((N+1, X_DIM))
                              self.u_sol = np.zeros((N, 1))
                              self.yref = np.zeros((N+1, COST_DIM))
                                self.solver.cost_set(i, "yref", self.yref[i])
                              self.solver.cost_set(N, "yref", self.yref[N][:COST_E_DIM])
                                self.solver.set(i, 'x', np.zeros(X_DIM))
                                self.solver.set(i, 'p', np.zeros(P_DIM))
                              self.solver.constraints_set(0, "lbx", x0)
                              self.solver.constraints_set(0, "ubx", x0)
                              self.solver.solve()
                              self.solve_time = 0.0
                              self.cost = 0
                    |self.reset_mpc(np.zeros(4))
                      def reset_mpc(self, x0=np.zeros(4)):
                        self.x0 = x0
                        self.lat_mpc.reset(x0=self.x0)
            |while True:
              sm.update()
              if sm.updated['modelV2']:
                |lateral_planner.update(sm)
                  def update(self, sm):
                    measured_curvature = sm['controlsState'].curvature
                    md = sm['modelV2']
                    if len(md.position.x) == TRAJECTORY_SIZE and len(md.orientation.x) == TRAJECTORY_SIZE:
                      self.path_xyz = np.column_stack([md.position.x, md.position.y, md.position.z])
                      self.plan_yaw = np.array(md.orientation.z)
                      self.plan_yaw_rate = np.array(md.orientationRate.z)
                      self.v_plan = np.clip(car_speed, MIN_SPEED, np.inf)
                      self.v_ego = self.v_plan[0]
                    self.lat_mpc.set_weights(PATH_COST, LATERAL_MOTION_COST,
                      LATERAL_ACCEL_COST, LATERAL_JERK_COST, STEERING_RATE_COST)
                    y_pts = self.path_xyz[:LAT_MPC_N+1, 1]
                    heading_pts = self.plan_yaw[:LAT_MPC_N+1]
                    yaw_rate_pts = self.plan_yaw_rate[:LAT_MPC_N+1]
                    p = np.column_stack([self.v_plan, lateral_factor])
                    self.lat_mpc.run(self.x0, p, y_pts, heading_pts, yaw_rate_pts)
                      # init state for next iteration
                      # mpc.u_sol is the desired second derivative of psi given x0 curv state.
                      # with x0[3] = measured_yaw_rate, this would be the actual desired yaw rate.
                      # instead, interpolate x_sol so that x0[3] is the desired yaw rate for lat_control.
                      self.x0[3] = measured_curvature * self.v_ego
                  def publish(self, sm, pm):
                    lateralPlan = plan_send.lateralPlan
                    lateralPlan.psis = self.lat_mpc.x_sol[0:CONTROL_N, 2].tolist()
                    lateralPlan.curvatures = (self.lat_mpc.x_sol[0:CONTROL_N, 3]/self.v_ego).tolist()
                    lateralPlan.curvatureRates = [float(x/self.v_ego) for x in self.lat_mpc.u_sol[0:CONTROL_N - 1]] + [0.0]
        |lateral_planner.publish(sm, pm)
--- Pipe 1a: plannerd.py < lateral_planner.py < lat_mpc.py < acados_ocp_solver_pyx.pyx < acados_solver.pxd
    < acados_solver.in.pxd < acados_solver.in.c < acados_ocp_solver_pyx.c < acados_solver_lat.c
--- Pipe 1b: solver = AcadosOcpSolverCython < cdef class AcadosOcpSolverCython
    < * cdef class AcadosOcpSolverCython: #
--- Pipe 1c: solver.solve() < cimport acados_solver, cdef class AcadosOcpSolverCython: def solve(self):
    < acados_solver.acados_solve(self.capsule)
--- Pipe 1d: acados_solver < int acados_solve "lat_acados_solve"(nlp_solver_capsule * capsule)
    < int acados_solve "{{ model.name }}_acados_solve"(nlp_solver_capsule * capsule)
    < int {{ model.name }}_acados_solve({{ model.name }}_solver_capsule* capsule)
      {int solver_status = ocp_nlp_solve(); return solver_status;}
    < ACADOS_SYMBOL_EXPORT int ocp_nlp_solve();
--- Pipe 1e: acados_solve() < ACADOS_SYMBOL_EXPORT int lat_acados_solve()
    < lat_acados_solve(){int solver_status = ocp_nlp_solve(); return solver_status }
--- Pipe 1f: lateral_planner.py > LateralPlanner > self.lat_mpc.reset(x0=self.x0) >
    def update() > measured_curvature = sm['controlsState'].curvature > md = sm['modelV2'] >
    self.path_xyz = ... md.position.x > self.plan_yaw = np.array(md.orientation.z) >
    self.lat_mpc.run(self.x0, p, y_pts, heading_pts, yaw_rate_pts) >
    self.x0[3] = measured_curvature * self.v_ego > def publish() >
    lateralPlan.psis = self.lat_mpc.x_sol > (..lat_mpc.x_sol[0:CONTROL_N, 3]..) >
    lateralPlan.curvatureRates = [..lat_mpc.u_sol[0:CONTROL_N - 1]] > pm.send('lateralPlan', )
--- Read controlsd.py > "measured_curvature" > lateral_planner.py
    /selfdrive/controls/controlsd.py
    |class Controls:
      def __init__(self, sm=None, pm=None, can_sock=None, CI=None):
        self.pm = messaging.PubMaster([... 'controlsState', ...])
        self.params = Params()
        self.sm = messaging.SubMaster([... 'liveParameters', ...]
        self.CI, self.CP = get_car()
          /selfdrive/car/car_helpers.py
            def get_car():
              CarInterface, CarController, CarState = interfaces[candidate]
              CP = CarInterface.get_params(candidate, fingerprints, car_fw, experimental_long_allowed)
              return CarInterface(CP, CarController, CarState), CP
--- 230213 Pipe 1 for get_car()
        self.LoC = LongControl(self.CP)
        |self.VM = VehicleModel(self.CP)
          /selfdrive/controls/lib/vehicle_model.py
            class VehicleModel:
              def __init__(self, CP: car.CarParams):
                self.cF_orig: float = CP.tireStiffnessFront
                self.cR_orig: float = CP.tireStiffnessRear
                  /selfdrive/car/toyota/interface.py
                    class CarInterface(CarInterfaceBase):
                      ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness()
                        /selfdrive/car/__init__.py
                          def scale_tire_stiffness(mass, wheelbase, center_to_front, tire_stiffness_factor=1.0):
                            center_to_rear = wheelbase - center_to_front
                            tir* mass / CivicParams.MASS * \
                                                   (center_to_rear / wheelbase) / (CivicParams.CENTER_TO_REAR / CivicParams.WHEELBASE)e_stiffness_front = (CivicParams.TIRE_STIFFNESS_FRONT * tire_stiffness_factor) ..
                            tire_stiffness_rear = (CivicParams.TIRE_STIFFNESS_REAR * tire_stiffness_factor) ..
                              #--- tireStiffnessFront, tireStiffnessRear = 118570.51 147271.0
                self.update_params(1.0, CP.steerRatio)
              def update_params(self, stiffness_factor: float, steer_ratio: float) -> None:
                self.cF: float = stiffness_factor * self.cF_orig
                self.cR: float = stiffness_factor * self.cR_orig
                self.sR: float = steer_ratio
        |self.LaC: LatControl
    |def step(self):
      CS = self.data_sample()
      |CC, lac_log = self.state_control(CS)
        def state_control(self, CS):
          lp = self.sm['liveParameters']
          x = max(lp.stiffnessFactor, 0.1)
          |sr = max(lp.steerRatio, 0.1)
            /selfdrive/locationd/paramsd.py
              def main(sm=None, pm=None):
                sm = messaging.SubMaster(['liveLocationKalman', 'carState'] ...)
                pm = messaging.PubMaster(['liveParameters'])
                min_sr, max_sr = 0.5 * CP.steerRatio, 2.0 * CP.steerRatio
                params['stiffnessFactor'] = 1.0
                learner = ParamsLearner(CP, params['steerRatio'], params['stiffnessFactor'], math.radians(params['angleOffsetAverageDeg']))
                  /selfdrive/locationd/paramsd.py
                    class ParamsLearner:
                      def __init__(self, CP, steer_ratio, stiffness_factor, angle_offset, P_initial=None):
                        self.kf = CarKalman(GENERATED_DIR, steer_ratio, stiffness_factor, angle_offset, P_initial)
                          /selfdrive/locationd/models/car_kf.py
                            class CarKalman(KalmanFilter):
                              /rednose_repo/rednose/helpers/kalmanfilter.py
                                class KalmanFilter:
                while True:
                  if sm.updated['liveLocationKalman']:
                    x = learner.kf.x
                    liveParameters.steerRatio = float(x[States.STEER_RATIO])
                    liveParameters.stiffnessFactor = float(x[States.STIFFNESS])
                    liveParameters.valid = all((
                      abs(liveParameters.angleOffsetDeg) < 10.0,
                      0.2 <= liveParameters.stiffnessFactor <= 5.0,
                      min_sr <= liveParameters.steerRatio <= max_sr,))
                    if sm.frame % 1200 == 0:  # once a minute
                      params = {
                        'steerRatio': liveParameters.steerRatio,
                        'stiffnessFactor': liveParameters.stiffnessFactor,
                        'angleOffsetAverageDeg': liveParameters.angleOffsetAverageDeg,}
                    pm.send('liveParameters', msg)
          |self.VM.update_params(x, sr)
      |self.publish_logs(CS, start_time, CC, lac_log)
        def publish_logs(self, CS, start_time, CC, lac_log):
            #"""Send actuators and hud commands to the car, send controlsstate and MPC logging"""
            # Curvature & Steering angle
          lp = self.sm['liveParameters']
          steer_angle_without_offset = math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg)
          |curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, lp.roll)
            /selfdrive/controls/lib/vehicle_model.py
              def calc_curvature(self, sa: float, u: float, roll: float) -> float:
                return (self.curvature_factor(u) * sa / self.sR) + self.roll_compensation(roll, u)
                  def curvature_factor(self, u: float) -> float:
                    sf = calc_slip_factor(self)
                      def calc_slip_factor(VM: VehicleModel) -> float:
                        return VM.m * (VM.cF * VM.aF - VM.cR * VM.aR) / (VM.l**2 * VM.cF * VM.cR)
                    return (1. - self.chi) / (1. - sf * u**2) / self.l
            # controlsState
          dat = messaging.new_message('controlsState')
          controlsState = dat.controlsState
          |controlsState.curvature = curvature
--- Pipe 2a: self.VM.update_params(x, sr) < self.VM = VehicleModel() < controlsd.py < VehicleModel
    < vehicle_model.py < ret.tireStiffnessFront, = scale_tire_stiffness() < /toyota/interface.py
    < def scale_tire_stiffness() < /car/__init__.py
--- Pipe 2b: kalmanfilter.py > KalmanFilter > car_kf.py > CarKalman(KalmanFilter) >
    paramsd.py > ParamsLearner > self.kf = CarKalman() > def main() > sm = messaging.SubMaster(..'liveLocationKalman') >
      learner = ParamsLearner() > x = learner.kf.x > liveParameters.steerRatio = float(x[States.STEER_RATIO]) >
      pm.send('liveParameters', )
--- Pipe 2c: controlsd.py > def state_control() > lp = self.sm['liveParameters'] >
    x = max(lp.stiffnessFactor,) > sr = max(lp.steerRatio,) > self.VM.update_params(x, sr) >
    def publish_logs() > steer_angle_without_offset = math.radians(..lp.angleOffsetDeg) >
    curvature = -self.VM.calc_curvature(steer_angle_without_offset,,) >
    controlsState.curvature = curvature > self.pm.send('controlsState',) >
    lateral_planner.py > measured_curvature = sm['controlsState'].curvature
--- Pipe 2d: 'liveLocationKalman' > KalmanFilter > CarKalman(KalmanFilter) > ParamsLearner >
    'liveParameters' > LateralPlanner
--- Read 'test_lateral_mpcJLL.py'
    def test_straight(self):
      x_sol, u_sol = run_mpc()
        def run_mpc(lat_mpc=None, v_ref=30., x_init=0., y_init=0., psi_init=0., curvature_init=0.,
          lat_mpc = LateralMpc()
          lat_mpc.run(x0, p, y_pts, heading_pts, curv_rate_pts)
--- Read 'lateral_planner.py'
    selfdrive/controls/lib/lateral_planner.py
      measured_curvature = sm['controlsState'].curvature
      # Parse model predictions
      md = sm['modelV2']
      if len(md.position.x) == TRAJECTORY_SIZE and len(md.orientation.x) == TRAJECTORY_SIZE:
        self.path_xyz = np.column_stack([md.position.x, md.position.y, md.position.z])
        self.t_idxs = np.array(md.position.t)
        self.plan_yaw = np.array(md.orientation.z)
        self.plan_yaw_rate = np.array(md.orientationRate.z)
        self.velocity_xyz = np.column_stack([md.velocity.x, md.velocity.y, md.velocity.z])
        car_speed = np.linalg.norm(self.velocity_xyz, axis=1)
        self.v_plan = np.clip(car_speed, MIN_SPEED, np.inf)
        self.v_ego = self.v_plan[0]
      self.x0[3] = interp(DT_MDL, self.t_idxs[:LAT_MPC_N + 1], self.lat_mpc.x_sol[:, 3])
        self.x0[3] = measured_curvature * self.v_ego
--- ToDo 1: Run selfdrive/modeld/tests/'test_modeld.py', cereal/messaging/tests/'test_pub_sub_master.py'

230530: OK MPC 1: PC: Run Read "mpc.py"
--- MPC 1: [Liu23b] + "mpc.py" A. Sakai
--- Run "mpc.py"
  (sconsvenv) jinn@Liu:~/openpilot/aMPC$ python mpc.py
--- OK: See /aMPC/Fig1MPC1.png - Fig1MPC3.png
--- Read "mpc.py"
    cx, cy, cyaw, ck = get_switch_back_course(dl)
      cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=dl)
        def calc_spline_course(x, y, ds=0.1):
          sp = CubicSpline2D(x, y)
            def __init__(self, x, y):
              self.s = self.__calc_s(x, y)
              self.sx = CubicSpline1D(self.s, x)
                def __init__(self, x, y):
                  self.a, self.b, self.c, self.d = [], [], [], []
                  self.x = x
                  self.y = y
              self.sy = CubicSpline1D(self.s, y)
          s = list(np.arange(0, sp.s[-1], ds))
            #--- s = [0.0, 1.0, 2.0, 3.0, ..., 100.0, 101.0, 102.0]
            #--- s = [0.0, 1.0, 2.0, 3.0, ..., 57.0, 58.0]
          for i_s in s:
            ix, iy = sp.calc_position(i_s)
              def calc_position(self, s):lateral_planner.update(sm)
                x = self.sx.calc_position(s)
                  def calc_position(self, x):
                    i = self.__search_index(x)
                    dx = x - self.x[i]
                    position = self.a[i] + self.b[i] * dx + \
                        self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0
                    return position
                y = self.sy.calc_position(s)
                return x, y
            rx.append(ix)
            ry.append(iy)
            ryaw.append(sp.calc_yaw(i_s))
            rk.append(sp.calc_curvature(i_s))
          return rx, ry, ryaw, rk, s
      cx, cy, cyaw, ck = get_switch_back_course(dl)
      cv = calc_speed_profile(cx, cy, cyaw, TARGET_SPEED)
      S0 = State(x=cx[0], y=cy[0], v=0.0, yaw=cyaw[0])  # initial state
      t, x, y, yaw, v, d, a = do_simulation(cx, cy, cyaw, ck, cv, dl, S0)
        def do_simulation(cx, cy, cyaw, ck, cv, dl, S0):
          target_ind, _ = calc_nearest_index(state, cx, cy, cyaw, 0)
          Sref, target_ind, dref = RefTrajectory(state, cx, cy, cyaw, ck, cv, dl, target_ind)
          oa, odelta, ox, oy, oyaw, ov = NewtonIter(Sref, Sk, dref, oa, odelta)
            def NewtonIter(Sref, Sk, dref, oa, od):
              Sbar = PredictState(Sk, oa, od, Sref)
              oa, od, ox, oy, oyaw, ov = LinearMPC(Sref, Sbar, Sk, dref)
                NS = 4  # S = x, y, v, yaw
                NU = 2  # U = [accel, steer]
                T = 5  # horizon length
                def LinearMPC(Sref, Sbar, Sk, dref):
                  S = cvxpy.Variable((NS, T + 1))
                  U = cvxpy.Variable((NU, T))
                  for t in range(T):
                    cost += cvxpy.quad_form(U[:, t], Rb)
                      cost += cvxpy.quad_form(Sref[:, t] - S[:, t], Q)
                    A, B, C = get_linear_model_matrix(Sbar[2, t], Sbar[3, t], dref[0, t])
                    constraints += [S[:, t + 1] == A @ S[:, t] + B @ U[:, t] + C]
                  prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
                  prob.solve(solver=cvxpy.ECOS, verbose=False)
              dU = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc U change value
          state = UpdateState(state, ai, di)
--- Pipe: CubicSpline1D() > CubicSpline2D() > get_switch_back_course() >
    do_simulation() > RefTrajectory() > NewtonIter() > LinearMPC()

230529: NG Ctr 1a: PID 1: AR 3: C2+body: Change "controlsd.py", "carcontroller.py"
--- Do Ctr 1a: Yaw < 0 => (0, 0, 0) => (X=0.63, Y=-0.8, Z=0) m. Want: (X=0, Y=0, Z=0)
--- XJ: Add self.speed_diff_calibrated to carcontroller.py due to wheelSpeeds.fl !=
      wheelSpeeds.fr in standing (CB turns even if actuators.steer=0. is set)
    self.speed_diff_calibrated = 0.  # XJ 7a: wheelSpeeds.fl != wheelSpeeds.fr in standing
    #turn_error = speed_diff_measured - speed_diff_desired  # manually tune
    turn_error = speed_diff_measured + self.speed_diff_calibrated  # auto tune
    self.speed_diff_calibrated = -speed_diff_measured
--- NG.
--- XJ: torque_l = torque - torque_diff - 2
--- NG.
--- PID 1: [Liu23a] + Apdx 1-4 CB5.py
--- ToDo 1: Do PID 1
--- ToDo 2: Ctr 1a w/ PID 1 is harder than I thought. It is suitable for student's project.
--- Back to XJ 6:
--- Ctr 3: CB rolled from (0,0) to (3.13,-0.15) back to (0.11, -0.17)m.
--- AR 3: CB moves from Point A to B with PID 1 in (X, Y) system.

230525: OK AR 2a: XJ 6: Ctr 3: Nav 1: AMN 1: C2+body: Change "controlsd.py", "carcontroller.py"
--- Nav 1: /selfdrive/assets/navigation/screenshot.png, GPSNMEAData,
    GpsLocationData, XYZTData, LiveLocationKalman, GnssMeasurements, UbloxGnss,
    QcomGnss, NavInstruction, NavRoute, NavModelData
--- Do AR 2a, CBQ 2, 230521 ToDo 2
--- XJ 6a: Change controlsd.py: Delete 230521 Pipe 3b, 3c: Delete these:
    self.LoC = LongControl(self.CP)
    self.LaC: LatControl
    self.VM = VehicleModel(self.CP)
    self.desired_curvature = 0.0
    self.desired_curvature_rate = 0.0
    self.experimental_mode = False
    self.v_cruise_helper = VCruiseHelper(self.CP)
    CC.cruiseControl.override = self.enabled and not CC.longActive and self.CP.openpilotLongitudinalControl
    CC.cruiseControl.cancel = CS.cruiseState.enabled and (not self.enabled or not self.CP.pcmCruise)
    CC.cruiseControl.resume = self.enabled and CS.cruiseState.standstill and speeds[-1] > 0.1
    from selfdrive.controls.lib.drive_helpers import VCruiseHelper, get_lag_adjusted_curvature
    from selfdrive.controls.lib.latcontrol import LatControl
    from selfdrive.controls.lib.longcontrol import LongControl
    from selfdrive.controls.lib.latcontrol_pid import LatControlPID
    from selfdrive.controls.lib.vehicle_model import VehicleModel
--- Run web.py
  root@localhost:/data/openpilot/tools/joystick$ python web.py
--- OK.
--- XJ 6b: use "distance_traveled":
    if abs(self.distance_traveled) < 3.0:  # XJ 6b for AR 2
      actuators.accel = 1.5*clip(0.5, -1, 1)
    torque_diff = 0.  # XJ 6b for AR 2
--- NG: CB rolls right
    speed_diff_desired = 0.  # XJ 6b for AR 2
--- NG: CB rolls right but better
    speed_diff_desired = -0.01  # XJ 6b for AR 2
--- OK: CB rolls straight but slowly turns left when standing
    speed_diff_desired = -0.005  # XJ 6b for AR 2
--- XJ 6c: I add self.odometer:
    self.odometer = 0                     # XJ 6c: cumulative distance traveled
    self.odometer += abs(CS.vEgo * DT_CTRL)  # XJ 6c
    if self.frame * DT_CTRL < 14.0:  # 14 seconds
        #if CS.vEgo > 0.:  # forward  # NG: it can be < 0.
      speed_diff_desired = -0.016  # XJ 6b for AR 2
    if self.sm.rcv_frame['testJoystick'] > 0:  # XJ 6c: for running web.py (remote control)
    if self.odometer < 3.0:  # XJ 6c: forward
    elif 3.0 <= self.odometer and self.odometer < 3.1:  # XJ 6c: standstill
    elif 3.1 <= self.odometer and self.odometer < 6.5:  # XJ 6c: backward
    else:  # XJ 6c: standstill
--- OK: CB rolls slightly right w/o turning when standing
--- Ctr 3: CB rolled grossly from (0,0) to (3.24, -0.57) m. =>
--- We need new PID with Path Model or GPS. =>
--- ToDo: AMN 1: AR 2 + Md 1 + Nav 1 (PID + Path + GPS)

========== Appendix (Apdx) ==========

Apdx 1. Journal: "Springer Autonomous Robots" reports on the theory and applications of
        robotic systems capable of some degree of self-sufficiency.
  A robot is a machine — especially one programmable by a computer — capable of
    carrying out a complex series of actions automatically.
  An autonomous robot is a robot that acts without recourse to human control.
  An actuator is a device that converts energy into motion. Actuators are used in
    almost every machine around us like electronic access control systems,
    mobile phone vibrators, household appliances, vehicles, robots & industrial devices.
    Robot Actuator : Types, Design, Working & Its Applications
  A sensor is a device that converts a physical signal into an electrical signal.
    Sensors turn a physical input into an electrical output, and actuators do the opposite.
    Physical signal includes heat, sound, pressure, flow, light or anything which is basically
    not electrical signal (voltage, current).

Apdx 2. carcontroller.py: PIDController, torque_r, comma_body.dbc: SG_ TORQUE_L,
  Electric Motors - Torque vs. Power and Speed
    Work is the result of a force acting over some distance.
      Work is quantified in Joules (J = Nm) or foot-pounds.
    Torque is a rotating force produced by a motor’s crankshaft.
      The more torque the motor produces, the greater is its ability to perform work.
      Since torque is a vector acting in a direction it is commonly quantified by
      the units J = Nm or pound-feet.
    Power is how rapidly work is accomplished - work in a given amount of time.
      Power is quantified in Watts (W = J/s) or horse power.

  https://www.powerelectric.com/motor-resources/motors101
    Speed versus Torque
      The output power of a motor sets the speed and torque performance boundaries of
      a motor, based on the equation:
    Power, Speed and Torque Relationship: Power (P) = Speed (n) x Torque (M)
      Since the rated output power of a motor is a fixed value, speed and torque are
      inversely related. As output speed increases, the available output torque decreases
      proportionately. As the output torque increases, the output speed decreases
      proportionately. This power, speed and torque relationship is commonly illustrated
      with a motor performance curve which often includes motor current draw (in Amps)
      and motor efficiency (in %).

  What is the relationship between motor speed and vehicle speed for a BLDC motor?
    Vehicle speed V = 3.6*RPM*r/(s*9.55) (km/h). Where s is gear ratio, RPM is
    rotational speed of the motor in revolutions per minute, r - wheel’s radius.
    For example s = 5:1, 1200 RPM and r = 0.32 we get V = 3.6* 1200*0.32/(10*9.55)
    = 14.48 km/h (acceleration at rated speed and full torque).
    Another example  5000 RPM and s = 5:1 gives 120.63 km/h — a highway cruising
    at full motor speed and low torque.

  DC Motor Controller: What It Is, Design Principles & Circuit Examples
    A motor controller ensures the efficient and safe operation of the motors.
    It can fulfill the following functions:
      start/stop the motor; change the rotation direction; control the speed and torque;
      provide overload protection; prevent electrical faults.

Apdx 3. (WR) Differential wheeled robot (two-wheeled robot)
  is a mobile robot whose movement is based on two separately driven wheels
  placed on either side of the robot body.

  2018 (WR) (PID) Two-wheeled robot platform based on PID control
  2011 (WR) Kinematics Equations for Differential Drive and Articulated
  2011 (WR) (PID) Dynamic modeling and control of a two wheeled robotic vehicle with a virtual payload
  2013 (WR) (PID) Simulation and Control of a Two-wheeled Self-balancing Robot
  2017 (WR) (Book) Wheeled mobile robotics: from fundamentals towards autonomous systems
  2013 (WR) (Rev) Review of modelling and control of two-wheeled robots
  2016 (WR) (Book) Springer handbook of robotics

Apdx 4. class LateralMpc():
  Real-time Model Predictive Control (MPC) with ACADO and PYTHON
  The ACADO toolkit will generate very fast MPC controllers (that perform one
    control step in microseconds) that can be used for realtime MPC control.
  The goal for the MPC controller is to autonomously steer a robot car on a
    reference course (reference trajectory).
  ACADO Toolkit for Automatic Control and Dynamic Optimization  https://acado.github.io/

  2018 (WR) (MPC) Model predictive speed and steering control
  2020 (WR) (MPC) acados a modular open-source framework for fast embedded optimal control
  2018 (WR) (Nav) PythonRobotics a Python code collection of robotics algorithms
       https://github.com/AtsushiSakai/PythonRobotics
  2000 (WR) (MPC) Tutorial overview of model predictive control
