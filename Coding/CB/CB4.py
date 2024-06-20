CB 4  Jinn-Liang Liu  2023.4.19 - 5.21  All Rights Reserved. © Copyright 2023

========== OP Coding Notes: Run, Change, Read ==========

"openpilot (OP)", "dragonpilot (DP)", "legacypilot (LP)",
"Comma Two (C2A/C2B)", and "Comma Body (CB)" Notes

OP/DP/LP Versions:
  1. DP091:   C2  Version 0.9.1 (2022-12-XX) downloaded on 2023-01-17
  2. OP091:   PC  Version 0.9.1 (2022-12-XX) downloaded on 2023-01-19
  3. DP091a:  PC  Version 0.9.1 (2023-02-23) downloaded on 2023-03-27
  4. DP091aL: PC  Long DP091a changed by JLL            on 230327
  5. DP2304:      from "DP091aL"                        on 230424 Deleted 230428
  6. LP092:       "legacypilot" by Rick Lan             on 230426

Abbreviations: CAN, Ctr, Md, Nav, UI
  AR (Autonomous Rolling), CAN (Control Area Network), CBQ (CB Question), Ctr (Control),
  Md (Model), Nav (Navigation), UI (User Interface), XJ (Changed by JLL),

========== Summary of Coding Notes ==========

230521: AR 2: C2+body: Read "self.LoC", "self.LaC", "PIDController"
--- AR 2: CB rolls a) forward/backward 3 m, b) forward/turn 180/forward/turn 180.
--- ToDo 1: Change from
      actuators.accel = 1.5*clip(self.sm['testJoystick'].axes[0], -1, 1)
--- to (w/o 'testJoystick')
      actuators.accel = self.LoC.update(CC.longActive, CS, long_plan, pid_accel_limits, t_since_plan)
--- Read "self.LoC", "self.LaC" from 230213 Read
--- Read [Liu23a]
--- Read "class CarController", "class PIDController()"
--- Q1: who calls actuators, hudControl, liveParameters ???
        are actuators = CC.actuators ... these == ???
    A1: Yes! actuators == CC.actuators  See controlsdJLL.py
--- carcontroller.py/deadband_filter(torque, deadband):
    What is the deadband in PID? Filtering?
--- Pipe 2: main() > Controls() > controlsd_thread() > step():
    CS = self.data_sample() > self.update_events(CS) > self.state_transition(CS) >
    CC = self.state_control(CS) > self.publish_logs(CS, , CC, lac_log)
--- Pipe 3: def state_control(CS) > CC = car.CarControl > actuators = CC.actuators >
    a) actuators.accel = 1.5*clip(), steer = clip(), actuators.steer = steer > return CC
    b) self.LoC > actuators.accel = self.LoC.update()
         longcontrol.py > def update() > self.pid.update() > pid.py > def update()
    c) self.LaC > actuators.steer = self.LaC.update() > return CC
         latcontrol.py > latcontrol_angle.py > def update()
--- Pipe 3: def self.publish_logs(,,CC,) > self.last_actuators = self.CI.apply(CC,) >
    d) /body/interface.py > def apply(, c,) > return self.CC.update(c,,) >
         /car/interfaces.py > CarInterfaceBase > self.CC = CarController() >
           /body/carcontroller.py > self.speed_pid, balance_pid, turn_pid = PIDController() >
             /lib/pid.py > def update()
           def update(,CC,,) > angle_setpoint = self.speed_pid.update(),
             torque = self.balance_pid.update(), torque_diff = self.turn_pid.update() >
             new_actuators = CC.actuators.copy() > new_actuators.accel = torque_l,
             new_actuators.steer = torque_r > return new_actuators
    > self.pm.send('carControl',)
--- Note: body (with controlsdXJ5.py) goes thru a) d) not b) c). cars thru b) c) d)

230509: OK CBQ 5, Ctr 2: XJ 5: C2+body: Change "controlsd.py"
--- OK: Road Test: C2B+car: no "Fan Malfunction" (Prius 12v Battery: 1.3kWh, 12V 4A)
--- C2B+body: "Fan Malfunction" (CB: 100Wh, 29.4V 2A; C2: 5V 2A) due to Panda ???
--- XJ 5a, 5c: /controlsd.py
--- XJ 5b: Delete C2/openpilot/selfdrive/car/chrysler etc.
--- Ctr 2a: -360° Yaw => (0, 0, 0) => (X=62.6, Y=-7.7, Z=0) cm in 7 m 6 s
--- Ctr 2b: "Joystick Mode" => "Green Ball" => body: Forward/Backward, Left/Right Turn
--- OK: Solved CBQ 5: "Fan Malfunction"

230506: NG. Install "LP092", "DP091" on C2A
--- Delete "DP091", Keep "LP092"
--- NG. See 211125 for OP079 on C2A with update.json of old NEOS

230505: OK UI 1: XJ 4: CBQ 5: C2+body: Change "onroad.cc"
--- XJ 4: Change Trim onroad.h, onroad.cc
--- XJ 4: Change (for "fuelGauge", "fuel_filter", "charging", "battery outline")
--- CBQ 5: "Fan Malfunction", "fanMalfunction"

230502: OK UI 1 CBQ 1: Data: XJ 1-3: CBQ 2-4: PC+C2+body: "realdata", Change "LP092XJ"
  root@localhost:/data/openpilot/tools/joystick$ python web.py
--- XJ 1: previous Changes
--- XJ 2: Trim "controlsd.py" to "controlsdXJ2.py"
--- XJ 3a: "Body Face": Line 58 C2/openpilot/selfdrive/ui/qt/home.cc
--- XJ 3b: Line 195 C2/openpilot/selfdrive/controls/controlsd.py
--- NG 1: Data: /data/media/0/realdata still logging
--- XJ 3c: Line 46 C2/openpilot/selfdrive/manager/process_config.py
--- CBQ 2: How to keep CB rolling straight w/o turning ???
--- CBQ 3: body On > body Off > C2 Off ???
--- UI 1a: C2: no "Battery fuelGauge"

230429: Ctr: PC+C2+body: Run "test_startup.py"
  (sconsvenv) jinn@Liu:~/openpilot$ python test_startup.py

230427: OK scons PC+C2: Install "LP092" (legacypilot) by Rick Lan
  root@localhost:/data/openpilot$ git submodule init
  root@localhost:/data/openpilot$ git submodule update
  root@localhost:/data/openpilot$ scons --clean
  root@localhost:/data/openpilot$ scons -j$(nproc)

230426: NG. PC: Install scons "legacypilot" (LP092) by Rick Lan
--- NG. Too many errors. Remove "legacypilot".

230425: OK. PC: NG. C2: scons "DP2304PC" as 230404. Trim scons "DP2304C2"
--- Change /openpilot to /DP091aL and /DP2304PC to /openpilot
  (sconsvenv) jinn@Liu:~/openpilot$ scons -u -j$(nproc)
--- OK.
  root@localhost:/data/openpilot$ scons -u -j$(nproc)
--- Error 2: C2 screen is "Black"

230424: Trim "DP091aL" to "DP2304PC = DP2304C2"

230422: NG: C2: Install OP0.8.13 - 0.9.1. OK: DP0.9.1, OP0.8.12
  root@localhost:/data/openpilot$ scons -j$(nproc)
--- C2: reboot

230420: NG. UI: PC+C2+body: "body.cc", "Body Face": /assets/body/awake.gif
--- Run C2/../_ui on PC
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
--- Run PC/../_ui_OP091 on C2
--- C2: reboot
--- Ccl A. C2/../_ui cannot run on PC and vice versa. (Ccl = Conclusion)
        B. We must build and run _ui solely on C2 or solely on PC.
        C. Install OP091 on C2+body and save C2/openpilot to openpilotDP091.

230419: OK CBQ 1: XJ 1-2: PC+C2+body: "controlsdXJ.py", XJ: Changed by JLL
  root@localhost:/data/openpilot/tools/joystick$ python web.py
--- XJ 1: previous Changes
--- XJ 2: Trim "controlsd.py" to "controlsdXJ.py"
--- XJ 3: Trim def state_control(self, CS):
--- CBQ 1: Why is CB so speedy when ignized, Why vibrates a lot, How to calibrate it,
--- Saved "controlsdXJ2.py", "controlsdXJ3.py"

===== Details of Coding Notes =====

230521: AR 2: C2+body: Read "self.LoC", "self.LaC", "PIDController"
--- See Ctr 1a, 1b
--- AR 2: CB rolls a) forward/backward 3 m, b) forward/turn 180/forward/turn 180.
--- Read "self.LoC", "self.LaC" from 230213 Read
    /selfdrive/controls/controlsd.py
    class Controls:
      def __init__(self, sm=None, pm=None, can_sock=None, CI=None):
        self.LoC = LongControl(self.CP)
          /controls/lib/longcontrol.py
          class LongControl:
            def __init__(self, CP):
              self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                /controls/lib/pid.py
                class PIDController():
                  def __init__(self, k_p, k_i, k_f=0., k_d=0., pos_limit=1e308, neg_limit=-1e308, rate=100):
                  def update(self, error, error_rate=0.0, speed=0.0, override=False, feedforward=0., freeze_integrator=False):
              self.last_output_accel = 0.0
            def update(self, active, CS, long_plan, accel_limits, t_since_plan):
              "Update longitudinal control. This updates the state machine and runs a PID loop"
              output_accel = self.pid.update(error_deadzone, speed=CS.vEgo,
              self.last_output_accel = clip(output_accel, accel_limits[0], accel_limits[1])
              return self.last_output_accel
        self.LaC: LatControl
          /controls/lib/latcontrol.py
          class LatControl(ABC):
            def __init__(self, CP, CI):
            @abstractmethod
            def update(self, active, CS, VM, params, last_actuators, steer_limited, desired_curvature, desired_curvature_rate, llk):
        self.LaC = LatControlAngle(self.CP, self.CI)
          /controls/lib/latcontrol_angle.py
          class LatControlAngle(LatControl):
            def __init__(self, CP, CI):
              super().__init__(CP, CI)
            def update(self, active, CS, VM, params, last_actuators, steer_limited, desired_curvature, desired_curvature_rate, llk):
              angle_log = log.ControlsState.LateralAngleState.new_message()
              angle_steers_des = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))
              angle_log.steeringAngleDesiredDeg = angle_steers_des
              return 0, float(angle_steers_des), angle_log
      def state_control(self, CS):
        CC = car.CarControl.new_message()
        actuators = CC.actuators
        actuators.accel = self.LoC.update(CC.longActive, CS, long_plan, pid_accel_limits, t_since_plan)
        actuators.steer = self.LaC.update(CC.latActive, CS, self.VM, lp,
        lac_log.output = actuators.steer
        return CC, lac_log
--- 230213 Pipe 1-2
--- Pipe 2: main() > Controls() > controlsd_thread() > step():
    CS = self.data_sample() > self.update_events(CS) > self.state_transition(CS) >
    CC = self.state_control(CS) > self.publish_logs(CS, , CC, lac_log)
--- Pipe 3: def state_control(CS) > CC = car.CarControl > actuators = CC.actuators >
    a) actuators.accel = 1.5*clip(), steer = clip(), actuators.steer = steer > return CC
    b) self.LoC > actuators.accel = self.LoC.update()
         longcontrol.py > def update() > self.pid.update() > pid.py > def update()
    c) self.LaC > actuators.steer = self.LaC.update() > return CC
         latcontrol.py > latcontrol_angle.py > def update()
--- Pipe 3: def self.publish_logs(,,CC,) > self.last_actuators = self.CI.apply(CC,) >
    d) /body/interface.py > def apply(, c,) > return self.CC.update(c,,) >
         /car/interfaces.py > CarInterfaceBase > self.CC = CarController() >
           /body/carcontroller.py > self.speed_pid, balance_pid, turn_pid = PIDController() >
             /lib/pid.py > def update()
           def update(,CC,,) > angle_setpoint = self.speed_pid.update(),
             torque = self.balance_pid.update(), torque_diff = self.turn_pid.update() >
             new_actuators = CC.actuators.copy() > new_actuators.accel = torque_l,
             new_actuators.steer = torque_r > return new_actuators
    > self.pm.send('carControl',)
--- Note: body (with controlsdXJ5.py) goes thru a) d) not b) c). cars thru b) c) d)
--- Read [Liu23a]
--- Read "class CarController", "class PIDController()"
--- carcontroller.py/deadband_filter(torque, deadband):
    What is the deadband in PID? Filtering?
    A deadband is a band of input values in the domain of a transfer function in a control
      system or signal processing system where the output is zero (the output is 'dead' -
      no action occurs).
    Measurement noise is particularly troublesome for a PID controller’s derivative action.
    The simplest solution to this problem is to reduce the derivative gain when
      measurement noise is high, but doing so limits its effectiveness.
    The measurement noise itself can sometimes be reduced by fixing the sensor or
      by filtering the process variable measurement mathematically.
    The effects of measurement noise can also be mitigated by simply ignoring insignificant
      changes in the sensor’s output. So long as the error between the process variable and
      the setpoint remains within deadband, the controller simply does nothing.
--- Q1: who calls actuators, hudControl, liveParameters ??? are these == ???
      actuators = CC.actuators, hudControl = CC.hudControl, liveParameters = msg.liveParameters
    A1: Yes! actuators == CC.actuators  See controlsdJLL.py
--- ToDo 1: Change from
    actuators.accel = 1.5*clip(self.sm['testJoystick'].axes[0], -1, 1)
--- to (w/o 'testJoystick')
    actuators.accel = self.LoC.update(CC.longActive, CS, long_plan, pid_accel_limits, t_since_plan)
--- ToDo 2: Change to (w/o 'testJoystick')
    actuators.accel = 1.5*clip(0.25, -1, 1)

230509: OK CBQ 5, Ctr 2: XJ 5: C2+body: Change "controlsd.py"
--- Solve CBQ 5: "Fan Malfunction", "fanMalfunction"
    /openpilot/selfdrive/controls/controlsd.py
      if (self.sm.frame - self.last_functional_fan_frame) * DT_CTRL > 15.0:
        self.events.add(EventName.fanMalfunction)
--- OK: Road Test: C2B+car: no "Fan Malfunction" (Prius 12v Battery: 1.3kWh, 12V 4A)
    The Toyota Prius has two batteries in it: a large 200-volt battery that is used
      to drive the car in the electric mode and a small 12-volt battery that
      is used to operate the accessories such as lights, radio, computers,
      ECU/inverter in communication etc.
    The prius does not have a “starter” but starts from the 220V hybrid battery.
--- C2B+body: "Fan Malfunction" (CB: 100Wh, 29.4V 2A; C2: 5V 2A) due to Panda ???
--- Change to
    if (self.sm.frame - self.last_functional_fan_frame) * DT_CTRL > 22.5:
--- NG
--- self.sm = messaging.SubMaster > self.sm.frame > SubMaster.self.frame > update_msgs() >
    self.frame += 1 > self.rcv_frame[s] = self.frame
--- Read and Run for CBQ 5 by controlsdJLL.py
    /cereal/messaging/socketmaster.cc
      cereal::Event::Reader &SubMaster::operator[](const char *name) const {
    /cereal/messaging/tests/test_pub_sub_master.py
--- XJ 5a: /controlsd.py
    #self.events.add(EventName.fanMalfunction)
    pass
--- OK: Solved CBQ 5: "Fan Malfunction"
--- XJ 5b: Delete C2/openpilot/selfdrive/car/chrysler etc.
--- XJ 5c: /controlsd.py
    #if not self.joystick_mode:
    #if CS.steeringPressed:
    #if lac_log.active and not recent_steer_pressed:
    #if self.CP.lateralTuning.which() == 'torque':
    #if self.joystick_mode:
    #if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
    #from selfdrive.controls.lib.latcontrol_indi import LatControlINDI
    #from selfdrive.controls.lib.latcontrol_torque import LatControlTorque
    #'driverMonitoringState','liveTorqueParameters'
    #resume_pressed, CS.gasPressed, CS.brakePressed, cruise_mismatch, self.cruise_mismatch_counter
    #recent_blinker, self.last_blinker_frame, ldw_allowed, hudControl
    #if not self.CP.openpilotLongitudinalControl:
    #self.last_steering_pressed_frame = 0
    #if not self.CP.experimentalLongitudinalAvailable or is_release_branch():
    #self.disengage_on_accelerator, self.CP.alternativeExperience, ALTERNATIVE_EXPERIENCE
    #LDW_MIN_SPEED, LANE_DEPARTURE_THRESHOLD, EventName.ldw, leftLaneDepart, ButtonType
    CC.latActive = self.active and (not standstill or self.joystick_mode)
    CC.longActive = self.enabled and self.CP.openpilotLongitudinalControl
--- Ctr 2a: -360° Yaw => (0, 0, 0) => (X=62.6, Y=-7.7, Z=0) cm in 7 m 6 s
--- Run web.py
    root@localhost:/data/openpilot/tools/joystick$ python web.py
--- => http://10.0.0.2:5000/ => "Green Ball" => body On =>
--- Error: "Process Not Running" "webjoystick"
--- C2 Power Off => C2 On => body On =>
--- Ctr 2: "Joystick Mode" > "Green Ball" > CB moves
--- OK.

230506: NG. Install "LP092", "DP091" on C2A
--- do 230427 to Install "LP092"
  root@localhost:/data$ git clone https://github.com/eFiniLan/legacypilot.git openpilot
  root@localhost:/data/openpilot$ scons --clean
--- Error 1
    scons: Reading SConscript files ...
    SConsEnvironmentError: No module named compilation_db:
      File "/data/openpilot/SConstruct", line 264:
        tools=["default", "cython", "compilation_db"],
--- Change to
        tools=["default", "cython"],
--- Error 2
    SConsEnvironmentError: No module named qt3:
      File "/data/openpilot/SConstruct", line 369:
        qt_env.Tool('qt3')
      File "/system/comma/usr/lib/python3.7/site-packages/scons/SCons/Environment.py", line 1798:
--- Delete "LP092"
--- Install "DP091"
  root@localhost:/data$ git clone https://github.com/dragonpilot-community/dragonpilot openpilot
  root@localhost:/data$ reboot
--- Error 3: C2 "Black"
--- rename to openpilotDP091X
--- Install "LP092"
  root@localhost:/data$ git clone https://github.com/eFiniLan/legacypilot.git openpilot
    Cloning into 'openpilot'...
    remote: Enumerating objects: 75046, done.
    remote: Counting objects: 100% (743/743), done.
    remote: Compressing objects: 100% (379/379), done.
    remote: Total 75046 (delta 446), reused 541 (delta 359), pack-reused 74303
    Receiving objects: 100% (75046/75046), 385.22 MiB | 2.37 MiB/s, done.
    Resolving deltas: 100% (46983/46983), done.
    Downloading selfdrive/legacy_modeld/models/dmonitoring_model.onnx (3.7 MB)
    Downloading selfdrive/legacy_modeld/models/dmonitoring_model_q.dlc (916 KB)
    Downloading selfdrive/legacy_modeld/models/supercombo.dlc (94 MB)
    Downloading selfdrive/legacy_modeld/models/supercombo.onnx (95 MB)
    Downloading selfdrive/modeld/models/dmonitoring_model.onnx (16 MB)
    Downloading selfdrive/modeld/models/dmonitoring_model_q.dlc (4.4 MB)
    Downloading selfdrive/modeld/models/navmodel.onnx (12 MB)
    Downloading selfdrive/modeld/models/navmodel_q.dlc (3.2 MB)
    Downloading selfdrive/modeld/models/supercombo.onnx (46 MB)
    Checking out files: 100% (2205/2205), done.
  root@localhost:/data/openpilot$ git submodule init
  root@localhost:/data/openpilot$ git submodule update
  root@localhost:/data/openpilot$ reboot
--- Error 3: C2 "Black"
  root@localhost:/data/openpilot$ scons -j$(nproc)
--- Error 1
--- Delete "DP091", Keep "LP092"
--- NG. See 211125 for OP079 on C2A with update.json of old NEOS

230505: OK UI 1: XJ 4: CBQ 5: C2+body: Change "onroad.cc"
--- Solve UI 1a: C2: no "Battery fuelGauge"
--- 230125 /openpilot/selfdrive/ui/qt/..
    /main.cc
    int main(int argc, char *argv[]) {
      MainWindow w;
        /window.cc
        MainWindow::MainWindow(QWidget *parent) : QWidget(parent) {
          main_layout = new QStackedLayout(this);
          homeWindow = new HomeWindow(this);
            /home.cc
            HomeWindow::HomeWindow(QWidget* parent) : QWidget(parent) {
              QHBoxLayout *main_layout = new QHBoxLayout(this);
              slayout = new QStackedLayout();
              main_layout->addLayout(slayout);
              onroad = new OnroadWindow(this);
                /onroad.cc
                OnroadWindow::OnroadWindow(QWidget *parent) : QWidget(parent) {
              slayout->addWidget(onroad);
              body = new BodyWindow(this);
                /body.cc
                BodyWindow::BodyWindow(QWidget *parent) : fuel_filter(1.0, 5., 1. / UI_FREQ), QWidget(parent) {
                  QStackedLayout *layout = new QStackedLayout(this);
                  face = new QLabel();
                  layout->addWidget(face);
                  awake = new QMovie("../assets/body/awake.gif");
              slayout->addWidget(body);
              QObject::connect(uiState(), &UIState::uiUpdate, this, &HomeWindow::updateState);
                void HomeWindow::updateState(const UIState &s) {
                  // switch to the generic robot UI
--- 230502 XJ 3a: "Body Face"
--- Change from
                  //slayout->setCurrentWidget(body);
--- to
                  slayout->setCurrentWidget(onroad);  // JLL
        main_layout->addWidget(homeWindow);
--- Q1: Who calls BodyWindow::paintEvent? see C++ 22
    4.9.1 Another example involving painting on PyQ5 widgets
      Every widget in QT5 has a method called paintEvent(…) that is called when
        the widget needs to be drawn (for instance, when its drawn for the first
        time or when the size of the widget has changed).
      We derive a new class from the respective widget class and override the
        paintEvent(…) method with our own implementation that takes care of the drawing.
--- Q2: BodyWindow::BodyWindow(QWidget *parent) : fuel_filter(1.0, 5., 1. / UI_FREQ), QWidget(parent) {
        C++ function definition: return_type function_name( parameter list ) { body of the function }
        C++, What does the colon after a constructor mean?
          It's an initialisation list for two things:
          1. Calling base class constructors
          2. Initialising member variables before executing the body of the constructor.
        Understanding Initialization Lists in C++
--- for /openpilot/selfdrive/ui/qt/body.cc
    /body.h
      FirstOrderFilter fuel_filter;
        /openpilot/common/util.h
          class FirstOrderFilter {
            FirstOrderFilter(float x0, float ts, float dt) {
            inline float update(float x) {
      bool charging = false;
    /cereal/messaging/messaging.h
      class SubMaster {
        SubMaster(const std::vector<const char *> &service_list, const std::vector<const char *> &poll = {},
        cereal::Event::Reader &operator[](const char *name) const;
        struct SubMessage;
        std::map<std::string, SubMessage *> services_;
          /cereal/messaging/socketmaster.cc
            struct SubMaster::SubMessage {
              cereal::Event::Reader event;
            SubMaster::SubMaster(const std::vector<const char *> &service_list, const std::vector<const char *> &poll,
            SubMessage *m = new SubMessage{
            m->msg_reader = new (m->allocated_msg_reader) capnp::FlatArrayMessageReader({});
            messages_[socket] = m;
            services_[name] = m;
            cereal::Event::Reader &SubMaster::operator[](const char *name) const {
    /cereal/gen/cpp/log.capnp.h
      inline  ::cereal::CarState::Reader getCarState() const;
    /openpilot/cereal/gen/cpp/car.capnp.h
      inline float CarState::Reader::getFuelGauge() const {
        return _reader.getDataField<float>(
          /openpilot/cereal/car.capnp
            fuelGauge @41 :Float32; # battery or fuel tank level from 0.0 to 1.0
--- XJ 4: Change Trim onroad.h, onroad.cc
--- XJ 4: Change (for "fuelGauge", "fuel_filter", "charging", "battery outline") from
    /body.h
    /body.cc
      BodyWindow::BodyWindow(QWidget *parent) : fuel_filter(1.0, 5., 1. / UI_FREQ), QWidget(parent) {
      void BodyWindow::paintEvent(QPaintEvent *event) {
        double fuel = std::clamp(fuel_filter.x(), 0.2f, 1.0f);
        if (charging) {
      void BodyWindow::offroadTransition(bool offroad) {
        fuel_filter.reset(1.0);
      void BodyWindow::updateState(const UIState &s) {
        const SubMaster &sm = *(s.sm);
        auto cs = sm["carState"].getCarState();
        charging = cs.getCharging();
        fuel_filter.update(cs.getFuelGauge());
--- to
    /onroad.h
      bool charging = false;  // from body.h
      FirstOrderFilter fuel_filter;  // from body.h
    /onroad.cc
      AnnotatedCameraWidget::AnnotatedCameraWidget(VisionStreamType type, QWidget* parent) : fps_filter(UI_FREQ, 3, 1. / UI_FREQ), fuel_filter(1.0, 5., 1. / UI_FREQ), CameraWidget("camerad", type, true, parent) {
      void AnnotatedCameraWidget::updateState(const UIState &s) {
      void AnnotatedCameraWidget::drawHud(QPainter &p) {
--- Change "battery outline" to
    p.translate(width() - 136, 50);
--- Trim "img_chffr_wheel.png"
    //QPoint center(btn_size / 2, btn_size / 2);
    //p.drawEllipse(center, btn_size / 2, btn_size / 2);
    //p.drawPixmap((btn_size - img_size) / 2, (btn_size - img_size) / 2, img);
--- Trim "US/Canada (MUTCD style) sign", "EU (Vienna style) sign"
--- Trim "dm icon", "experimental_btn", "ExperimentalButton"
--- OK UI 1a but
--- CBQ 5: "Fan Malfunction", "fanMalfunction"

230502: OK UI 1 CBQ 1: Data: XJ 1-3: CBQ 2-4: PC+C2+body: "realdata", Change "LP092XJ"
--- redo 230419
--- XJ 1: previous Changes
    Line 71, 72, 77 C2/data/openpilot/selfdrive/manager/process_config.py
      #PythonProcess("updated", "selfdrive.updated", enabled=not PC, onroad=False, offroad=True),
      #PythonProcess("uploader", "system.loggerd.uploader", offroad=True),
      PythonProcess("webjoystick", "tools.joystick.web", onroad=False, callback=notcar),
    Line 5 in /openpilot/tools/joystick/joystickd.py to
      from inputs import get_gamepad
    Line 618 /selfdrive/controls/controlsd.py:
      actuators.accel = 1.5*clip(self.sm['testJoystick'].axes[0], -1, 1)
--- Run body
  root@localhost:/data/openpilot/tools/joystick$ python web.py
--- C2 Power Off > C2 On > body On
--- OK: Solved CBQ 1 by OP092
--- Read for CBQ 1 Solved by OP092
--- C2 Alert: "Invalid date and time settings"
--- CBQ 2: How to keep CB rolling straight w/o turning ???
--- XJ 2: Trim "controlsd.py" to "controlsdXJ2.py"
    #REPLAY = "REPLAY" in os.environ
    #SIMULATION = "SIMULATION" in os.environ
    #LaneChangeState = log.LateralPlan.LaneChangeState
    #LaneChangeDirection = log.LateralPlan.LaneChangeDirection
    #"driverCameraState"
    #def set_initial_state(self):
--- OK
--- CBQ 3: body On > body Off > C2 Off ???
--- try controlsdXJ1.py for CBQ 3 > body On > web.py > Error (must be body Off)
    > body Off > C2 Off > CBQ 3 (same as XJ2)
--- re-try controlsdXJ2.py > body On > body OK > body Off > C2 Off > CBQ 3
--- XJ: "Body Face": Trim C2/openpilot/selfdrive/ui/qt/body.cc
    #face = new QLabel();
    #if (m != face->movie()) {
--- reboot
--- NG: C2 Screen "Black". undo XJ
--- XJ 3a: "Body Face": Line 58 C2/openpilot/selfdrive/ui/qt/home.cc
    //slayout->setCurrentWidget(body);
    slayout->setCurrentWidget(onroad);  // JLL
--- OK: C2: "Joystick Mode" but no "Battery fuelGauge"
--- "Joystick Mode": joystick_mode, JoystickDebugMode, joystickDebug, joystick_alert, testJoystick
--- XJ: Line 343 C2/openpilot/selfdrive/controls/lib/events.py
    #ET.PERMANENT: NormalPermanentAlert("Joystick Mode"),
--- NG.
--- XJ: Line 342 C2/openpilot/selfdrive/controls/lib/events.py
    #ET.WARNING: joystick_alert,
--- NG.
--- XJ 3b: Line 195 C2/openpilot/selfdrive/controls/controlsd.py
    #self.events.add(EventName.joystickDebug, static=True)
--- OK but
--- UI 1a: C2: no "Battery fuelGauge"
--- NG 1: Data: /data/media/0/realdata still logging
--- XJ 3c: Line 46 C2/openpilot/selfdrive/manager/process_config.py
    #NativeProcess("loggerd", "selfdrive/loggerd", ["./loggerd"], onroad=False, callback=logging),
--- NG 1, CBQ 3, OK: Solved UI 1 on C2+body

230429: Ctr: PC+C2+body: Run "test_startup.py"
--- PC: Run /openpilot/selfdrive/controls/tests/test_startup.py
  (sconsvenv) jinn@Liu:~/openpilot$ python test_startup.py
    Waiting for CAN messages...
    Using cached CarParams
    /home/jinn/openpilot/system/swaglog.py:77: ResourceWarning: unclosed socket <zmq.Socket(zmq.PUSH) at 0x7f6b6f79eb20>
      self.sock = self.zctx.socket(zmq.PUSH)
    ResourceWarning: Enable tracemalloc to get the object allocation traceback
    VIN 11111111111111111
    {"event": "fingerprinted", "car_fingerprint": "TOYOTA COROLLA 2017", "source": 1, "fuzzy": false, "cached": true, "fw_count": 6, "ecu_responses": [], "vin_rx_addr": 0, "error": true}
    SIGINT received, exiting
    .Waiting for CAN messages...
    Setting OBD multiplexing to True
    OBD multiplexing set successfully
    vin query retry (1) ...
    vin query retry (5) ...
    Setting OBD multiplexing to False
    OBD multiplexing set successfully
    VIN 00000000000000000
    {"event": "fingerprinted", "car_fingerprint": null, "source": 0, "fuzzy": false, "cached": false, "fw_count": 0, "ecu_responses": [], "vin_rx_addr": 0, "error": true}
    {"event": "car doesn't match any fingerprints", "fingerprints": {"0": {"1": 1, "2": 1, "3": 1, "4": 1, "5": 1, "6": 1, "7": 1, "8": 1, "9": 1, "10": 1, "11":
    .Waiting for CAN messages...
    Getting VIN & FW versions
    ResourceWarning: Enable tracemalloc to get the object allocation traceback
    Fingerprinted TOYOTA COROLLA 2017 using fuzzy match. 2 matching ECUs
    VIN 11111111111111111
    {"event": "fingerprinted", "car_fingerprint": "TOYOTA COROLLA 2017", "source": 1, "fuzzy": true, "cached": true, "fw_count": 6, "ecu_responses": [], "vin_rx_addr": 0, "error": true}
    SIGINT received, exiting
    ----------------------------------------------------------------------
    Ran 10 tests in 47.846s
    OK

230427: OK scons PC+C2: Install "LP092" (legacypilot) by Rick Lan
  root@localhost:/data$ git clone https://github.com/eFiniLan/legacypilot.git openpilot
  root@localhost:/data/openpilot$ git submodule init
  root@localhost:/data/openpilot$ git submodule update
  root@localhost:/data/openpilot$ scons --clean
    Removed /tmp/scons_cache/config
    Removed directory /tmp/scons_cache
  root@localhost:/data/openpilot$ scons -j$(nproc)
--- Error 1
    opendbc/can/dbc.cc:262:20: error: no member named 'directory_iterator' in
          namespace 'filesystem'
      for (filesystem::directory_iterator i(dbc_file_path), end; i != end; i++) {
           ~~~~~~~~~~~~^
--- FTP /jinn/DP091aL/opendbc/can/dbc.cc to C2
--- Error 2
    opendbc/can/dbc.cc:207:10: error: call to 'dbc_parse_from_stream' is ambiguous
      return dbc_parse_from_stream(dbc_name, infile, checksum.get());
             ^~~~~~~~~~~~~~~~~~~~~
--- FTP /commaai/OP092/opendbc/can/dbc.cc to C2
--- Error 3
    /data/data/com.termux/files/usr/bin/aarch64-linux-android-ld: opendbc/can/dbc.os: in function `std::__ndk1::__fs::filesystem::path::filename() const':
    /system/comma/usr/bin/../include/c++/v1/filesystem:1096: undefined reference to `std::__ndk1::__fs::filesystem::path::__filename() const'
    clang-8: error: linker command failed with exit code 1 (use -v to see invocation)
    scons: *** [opendbc/can/libdbc.so] Error 1
--- use old dbc.cc (before today) sent by Wang
--- Wang: OK on EON
--- OK.  /dev/block/sda10   24G  6.0G   18G  26% /data
230427b: OK. PC: Install scons "legacypilot" (LP092) by Rick Lan
  jinn@Liu:~$ git clone https://github.com/eFiniLan/legacypilot.git openpilot
  jinn@Liu:~/openpilot$ git submodule init
  jinn@Liu:~/openpilot$ git submodule update
  (sconsvenv) jinn@Liu:~/openpilot$ scons --clean
    Removed /tmp/scons_cache/config
    Removed directory /tmp/scons_cache
  (sconsvenv) jinn@Liu:~/openpilot$ scons -j$(nproc)
--- Error 1
    opendbc/can/dbc.cc:271:4: error: #else without #if
      #else
       ^
    opendbc/can/dbc.cc:272:2: error: use of undeclared identifier 'filesystem'; did you mean 'std::filesystem'?
            filesystem::recursive_directory_iterator beg_iter(dbc_file_path), end_iter;
--- new dbc.cc from https://github.com/eFiniLan/legacypilot
--- Error 2
    selfdrive/boardd/boardd.cc:417:8: error: no member named 'setFanStallCount' in 'cereal::PandaState::Builder'
        ps.setFanStallCount(health.fan_stall_count);
        ~~ ^
--- Change Line 417 /openpilot/selfdrive/boardd/boardd.cc to
  //    ps.setFanStallCount(health.fan_stall_count);
--- Error 3
    In file included from selfdrive/legacy_modeld/models/dmonitoring.cc:6:
    ./common/legacy_modeldata.h:34:15: error: no member named 'EON' in 'HardwarePC'
        Hardware::EON() ? (mat3){{910., 0., 1164.0 / 2,
        ~~~~~~~~~~^
--- Change /openpilot/common/legacy_modeldata.h to
    https://github.com/commaai/openpilot/blob/master/common/modeldata.h
--- add Lines 26-33 to legacy_modeldata.h from legacy_modeldata_LP092.h
--- Error 4
    selfdrive/loggerd/bootlog.cc:14:24: error: no member named 'EON' in 'HardwarePC'
      } else if (Hardware::EON()) {
                 ~~~~~~~~~~^
    selfdrive/loggerd/logger.cc:39:17: error: no member named 'EON' in 'HardwarePC'
      if (Hardware::EON()) {
          ~~~~~~~~~~^
--- add Line 11 to /openpilot/system/hardware/pc/hardware.h
    static bool EON() { return false; }
--- Error 5
    scons: *** [selfdrive/loggerd/logger_util.o] Source `selfdrive/ui/replay/util.cc' not found, needed by target `selfdrive/loggerd/logger_util.o'.
--- Change Line 33 /openpilot/selfdrive/loggerd/SConscript to
    openpilot/tools/replay
--- Change Line 12 /openpilot/selfdrive/loggerd/tests/test_logger.cc to
    #include "../../../tools/replay/util.h"
--- Error 6
    selfdrive/ui/qt/offroad/settings.cc:333:62: error: use of undeclared identifier 'HardwareEon'; did you mean 'HardwareNone'?
      QObject::connect(wifiBtn, &ButtonControl::clicked, [=]() { HardwareEon::launch_wifi(); });
--- Change /openpilot/selfdrive/ui/qt/offroad/settings.cc
    Lines 333-335
    #ifdef QCOM
    QObject::connect(wifiBtn, &ButtonControl::clicked, [=]() { HardwareEon::launch_wifi(); });
    #endif
    Lines 339-341                                                                 HardwareNone
    #ifdef QCOM
    QObject::connect(tetheringBtn, &ButtonControl::clicked, [=]() { HardwareEon::launch_tethering(); });
    #endif
--- Error 7
    Scanning directory '/home/jinn/openpilot/selfdrive/ui'...
    lupdate error: File '/home/jinn/openpilot/selfdrive/ui/translations/main_en.ts-I' has no recognized extension.
    Traceback (most recent call last):
      File "selfdrive/ui/update_translations.py", line 46, in <module>
        update_translations(args.vanish, args.plural_only)
      File "selfdrive/ui/update_translations.py", line 33, in update_translations
        assert ret == 0
    AssertionError
--- Change Line 27 /openpilot/selfdrive/ui/update_translations.py to                                                        ^~~~~~~~~~~
    args = f"lupdate -locations none -recursive {UI_DIR} -ts {tr_file}"
--- OK.
230427a: PC: C2: Change scons "DP2304C2"
--- SConstruct091 Lines 301-309, 314, 334: compared to SConstruct0812
--- SConscriptUI091 "ui.cc": compared to SConscriptUI0812
--- Change and save SConscriptUI091 to SConscriptUI2304 with SConscriptUI0812's "ui.cc"
--- FTP /openpilot/selfdrive/ui/SConscript2304 to C2
  root@localhost:/data/openpilot$ scons --clean
  root@localhost:/data/openpilot$ scons -j$(nproc)
--- Error 1
    /system/comma/usr/bin/../lib/gcc/arm-none-eabi/4.7.1/../../../../arm-none-eabi/bin/ld: cannot find -lgcc
    collect2: error: ld returned 1 exit status
    scons: *** [body/board/obj/bootstub.body.elf] Error 1
--- ***Note: C2/openpilotDP091 does not have /body
--- Change SConstruct2304 Line 413
--- FTP SConstruct2304 to C2
  root@localhost:/data/openpilot$ scons -j$(nproc)
--- Error 2
    scons: *** [cereal/gen/c/include/c++.capnp.h] Error 1
--- FTP /jinn/openpilot/cereal/gen/c/include/c++.capnp.h to C2
--- Error 2
    scons: *** [cereal/gen/c/include/c++.capnp.h] Error 1
--- ***Note: C2/openpilotDP091 does not have /cereal/gen
--- Change SConstruct2304 Line 361, 414
  root@localhost:/data/openpilot$ scons --clean
--- Error 3
    scons: *** [common/clock.cpp] Error 1
--- ToDo: SConstruct2304: Change SConstruct091 Lines 301-309, 314, 334 to those in SConstruct0812
--- NG.

230427 Question for Rick:
  230115: OK Install: Dragonpilot DPbeta2 = DP091 on C2+body/car
  --- Version 0.9.1 (2022-12-XX) at https://smiskol.com/fork/dp/beta2
  All SConstruct and SConscript files are missing in this fork version.
  Do you still have those files?

230426a: NG. PC: Install scons "legacypilot" by Rick Lan
  jinn@Liu:~$ git clone https://github.com/eFiniLan/legacypilot.git
  jinn@Liu:~/legacypilot$ git submodule init
  jinn@Liu:~/legacypilot$ git submodule update
--- rename legacypilot to openpilot
  (sconsvenv) jinn@Liu:~/openpilot$ scons -u -j$(nproc)
--- Error 1
    opendbc/can/dbc.cc:217:32: error: use of undeclared identifier 'filesystem'; did you mean 'std::filesystem'?
--- rename /openpilot/opendbc/can/dbc.cc, common_dbc.h to /dbc_OP092.cc, common_dbc_OP092.h
--- Copy /openpilotX/../dbc.cc, common_dbc.h  to /openpilot/..
--- Error 2
    opendbc/can/parser.cc:328:9: error: no member named 'ts_nanos' in 'SignalValue'
--- rename /openpilot/opendbc/can/parser.cc to /parser_OP092.cc
--- Copy /openpilotX/../parser.cc  to /openpilot/..
--- Error 3
    opendbc/can/parser.cc:302:37: error: out-of-line definition of 'query_latest' does not match any declaration in 'CANParser'
    std::vector<SignalValue> CANParser::query_latest() {
                                      ^~~~~~~~~~~~
--- rename to /openpilot/opendbc/can_OP092
--- Copy /openpilotX/../can to /openpilot/..
--- Error 3
    selfdrive/boardd/boardd.cc:417:8: error: no member named 'setFanStallCount' in 'cereal::PandaState::Builder'
--- rename to /openpilot/selfdrive/boardd_OP092
--- Copy /openpilotX/.. to /openpilot/..
--- Error 4
    scons: *** [selfdrive/boardd/tests/test_boardd_usbprotocol.o] Source `selfdrive/boardd/tests/test_boardd_usbprotocol.cc' not found, needed by target `selfdrive/boardd/tests/test_boardd_usbprotocol.o'.
--- Copy /openpilot/selfdrive/boardd_OP092/tests to /openpilot/..
--- Error 5
    In file included from selfdrive/legacy_modeld/models/dmonitoring.cc:6:
    ./common/legacy_modeldata.h:34:15: error: no member named 'EON' in 'HardwarePC'
--- rename to /openpilot/selfdrive/legacy_modeld/models_OP092
--- Copy /openpilotX/.. to /openpilot/..
--- Error 6
    scons: *** [selfdrive/loggerd/logger_util.o] Source `selfdrive/ui/replay/util.cc' not found, needed by target `selfdrive/lo
--- Change /openpilot/selfdrive/loggerd/SConscript to
    #if GetOption('test'):
    #  env.Program('tests/test_logger', ['tests/test_runner.cc', 'tests/test_loggerd.cc', 'tests/test_logger.cc', env.Object('logger_util', '#/selfdrive/ui/replay/util.cc')], LIBS=libs + ['curl', 'crypto'])
--- Error 7
    selfdrive/legacy_modeld/dmonitoringmodeld.cc:24:5: error: unknown type name 'DMonitoringResult'; did you mean 'DMonitoringModelResult'?
--- Copy /openpilotX/selfdrive/modeld/dmonitoringmodeld.cc to /openpilot/..
--- Error 8
    selfdrive/loggerd/bootlog.cc:14:24: error: no member named 'EON' in 'HardwarePC'
--- rename to /openpilot/selfdrive/loggerd_OP092
--- Error 9
    scons: *** [selfdrive/legacy_modeld/_dmonitoringmodeld] Error 1
    selfdrive/legacy_modeld/models/driving.cc:203:16: error: no member named 'XYZTData' in 'cereal::ModelDataV2'; did you mean
--- rename to /openpilot/selfdrive/legacy_modeld/models_OP091
--- Error 10
    scons: *** [selfdrive/loggerd/tests/test_runner.o] Source `selfdrive/loggerd/tests/test_runner.cc' not found, needed by tar
--- Change /openpilot/selfdrive/loggerd/SConscript to
    #if GetOption('test'):
    #  env.Program('tests/test_logger', ['tests/test_runner.cc', 'tests/test_loggerd.cc', 'tests/test_logger.cc', env.Object('logger_util', '#/selfdrive/ui/replay/util.cc')], LIBS=libs + ['curl', 'crypto'])
--- Error 11
    In file included from selfdrive/legacy_modeld/models/dmonitoring.cc:6:
    ./common/legacy_modeldata.h:34:15: error: no member named 'EON' in 'HardwarePC'
--- NG. Too many errors.

230426: NG. PC: Install scons "legacypilot" (LP092) by Rick Lan
--- (Version 0.9.2 (2023-03-XX))
  jinn@Liu:~$ git clone https://github.com/eFiniLan/legacypilot.git
  jinn@Liu:~$ du -hsc /home/jinn/legacypilot
    1.2G	/home/jinn/legacypilot
--- rename legacypilot to openpilot
  (sconsvenv) jinn@Liu:~/openpilot$ scons -u -j$(nproc)
--- Error 1
    Missing SConscript 'cereal/SConscript'
    File "/home/jinn/openpilot/SConstruct", line 416, in <module>
    scons: warning: Ignoring missing SConscript 'rednose/SConscript'
    File "/home/jinn/openpilot/SConstruct", line 453, in <module>
    scons: warning: Ignoring missing SConscript 'cereal/SConscript'
    File "/home/jinn/openpilot/SConstruct", line 468, in <module>
    scons: warning: Ignoring missing SConscript 'opendbc/can/SConscript'
    File "/home/jinn/openpilot/SConstruct", line 468, in <module>
    scons: warning: Ignoring missing SConscript 'panda/SConscript'
    File "/home/jinn/openpilot/SConstruct", line 468, in <module>
    scons: *** Import of non-existent variable ''libkf''
    File "/home/jinn/openpilot/selfdrive/locationd/SConscript", line 1, in <module>
--- Copy /openpilotX/cereal to /openpilot/.. etc.
  (sconsvenv) jinn@Liu:~/openpilot$ scons -u -j$(nproc)
--- Error 2
    ModuleNotFoundError: No module named 'laika'
    scons: *** [selfdrive/locationd/models/generated/loc_4.cpp] Error 1
--- Copy /openpilotX/laika to /openpilot/..
--- Error 3
    selfdrive/boardd/boardd.cc:393:8: error: no member named 'setVoltage' in 'cereal::PandaState::Builder'
        ps.setVoltage(health.voltage_pkt);
        ~~ ^
    selfdrive/boardd/boardd.cc:394:8: error: no member named 'setCurrent' in 'cereal::PandaState::Builder'
        ps.setCurrent(health.current_pkt);
        ~~ ^
    selfdrive/boardd/boardd.cc:417:8: error: no member named 'setFanStallCount' in 'cereal::PandaState::Builder'
        ps.setFanStallCount(health.fan_stall_count);
        ~~ ^
    selfdrive/boardd/boardd.cc:417:32: error: no member named 'fan_stall_count' in 'health_t'
        ps.setFanStallCount(health.fan_stall_count);
                            ~~~~~~ ^
    selfdrive/boardd/boardd.cc:421:8: error: no member named 'setSpiChecksumErrorCount' in 'cereal::PandaState::Builder'
        ps.setSpiChecksumErrorCount(health.spi_checksum_error_count);
        ~~ ^
    selfdrive/boardd/boardd.cc:421:40: error: no member named 'spi_checksum_error_count' in 'health_t'
        ps.setSpiChecksumErrorCount(health.spi_checksum_error_count);
                                    ~~~~~~ ^
--- rename /openpilot/selfdrive/boardd/boardd.cc to boardd_OP092.cc
--- Copy /openpilotX/../boardd.cc to /openpilot/..
--- Error 4
    In file included from selfdrive/legacy_modeld/models/dmonitoring.cc:6:
    ./common/legacy_modeldata.h:36:15: error: no member named 'EON' in 'HardwarePC'
        Hardware::EON() ? (mat3){{910., 0., 1164.0 / 2,
        ~~~~~~~~~~^
--- rename /openpilot/common/legacy_modeldata.h to /legacy_modeldata_OP092.cc
--- rename /openpilot/common/modeldata.h to /legacy_modeldata.h
--- Error 5
    selfdrive/loggerd/logger.cc:39:17: error: no member named 'EON' in 'HardwarePC'
--- rename /openpilot/selfdrive/loggerd/logger.cc to /logger_OP092.cc
--- rename /openpilot/selfdrive/loggerd/logger.h to /logger_OP092.h
--- Copy /openpilotX/../logger.cc, logger.h  to /openpilot/..
--- Error 6
    selfdrive/loggerd/bootlog.cc:14:24: error: no member named 'EON' in 'HardwarePC'
--- Error 7
    selfdrive/legacy_modeld/models/dmonitoring.cc:70:16: error: use of undeclared identifier 'TICI_CAM_WIDTH'
--- Error 8
    selfdrive/legacy_modeld/dmonitoringmodeld.cc:24:5: error: unknown type name 'DMonitoringResult'; did you mean 'DMonitoringModelResult'?
--- Error 9
    selfdrive/legacy_modeld/models/dmonitoring.cc:6:10: fatal error: 'common/modeldata.h' file not found
--- Error 10
    selfdrive/loggerd/loggerd.cc:104:61: error: no member named 'u' in 'VisionBuf'
--- Error 11
    ./selfdrive/loggerd/loggerd.h:26:10: fatal error: 'selfdrive/loggerd/encoder/encoder.h' file not found
--- Error 12
    ./selfdrive/loggerd/encoder/encoder.h:10:10: fatal error: 'selfdrive/loggerd/video_writer.h' file not found
--- Error 13
    selfdrive/loggerd/main.cc:6:17: error: no member named 'EON' in 'HardwarePC'
--- remove /openpilot/selfdrive/loggerd
--- Copy /openpilotX/../loggerd to /openpilot/..
--- Error 14
    selfdrive/legacy_modeld/models/driving.cc:195:16: error: no member named 'XYZTData' in namespace 'cereal'; did you mean 'cereal::ModelDataV2::XYZTData'?
--- NG. Too many errors. Remove "legacypilot".

230426 Question for Rick:
  230425: PC: scons "DP2304". OK. C2: scons "DP2304". NG.
  I can scons DP2304 on PC but not C2. Why?
  Please see the following details:
  DP/OP Versions:
  OP091: Version 0.9.1 (2022-12-XX) downloaded on 2023-01-19 from https://github.com/commaai/openpilot
  DP091: Version 0.9.1 (2023-02-23) downloaded on 2023-03-27 from
  https://github.com/dragonpilot-community/dragonpilot
  DP2304 =  DP091 + OP091 with changes by me

230425: OK. PC: NG. C2: scons "DP2304PC" as 230404. Trim scons "DP2304C2"
--- Change /openpilot to /DP091aL and /DP2304PC to /openpilot
  (sconsvenv) jinn@Liu:~/openpilot$ scons -u -j$(nproc)
--- OK.
  jinn@Liu:~$ du -hsc /home/jinn/openpilot
    3.3G	/home/jinn/openpilot
--- Copy /DP091aL/uiviewJLL.py to /openpilot/..
--- Copy /DP091aL/tools/replay/replayJLL to /openpilot/..
--- move /home/jinn/dataC to /openpilot/tools/replay/dataC
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
  (sconsvenv) jinn@Liu:~/openpilot/tools/replay$
    ./replayJLL --data_dir dataC "8bfda98c9c9e4291|2020-05-11--03-00-57--61"
--- OK.
--- Free space /home: 5.7 GB
--- upload Old OP versions to gapp.nthu.edu.tw: Failed: You are offline.
--- Old OPs to /media/jinn/Liu/OP_Backup
--- Free space /home: 11.2 GB
--- compare /DP2304C2 to /OP/DP091a,
--- Trim "DP2304C2"
    Move to Trash: /DP2304C2/tools, /DP2304C2/selfdrive/car/chrysler etc.,
      /DP2304C2/selfdrive/ui/_ui, _ui_nonav etc., /ui, ui_DP091C2b etc.
    rename /DP2304C2/../ui_DP091C2a to /ui
    move /OP/DP091a/tools to /DP2304C2/..
    search "_DP" in /DP2304C2
--- FTP "DP2304C2" to C2
--- rename openpilot to openpilotDP091
--- rename DP2304C2 to openpilot
  root@localhost:/data/openpilot$ scons -u -j$(nproc)
--- Error 1
    /system/comma/usr/bin/../lib/gcc/arm-none-eabi/4.7.1/../../../../arm-none-eabi/bin/ld: cannot find -lgcc
    collect2: error: ld returned 1 exit status
    scons: *** [body/board/obj/bootstub.body.elf] Error 1
  root@localhost:/data/openpilot$ reboot
--- Error 2: C2 screen is "Black"
--- FTP C2/openpilotDP091/selfdrive/ui/_ui (27.6 MB), _ui_nonav (25.0 MB) to PC/DP2304C2/../_ui, _ui_nonav
--- FTP PC/DP2304C2/../_ui, _ui_nonav to C2/openpilot/../_ui, _ui_nonav
--- reboot
--- Error 2: C2 screen is "Black"
--- NG.
--- rename openpilot to openpilotX
--- clone OP091 (redo OP091 from git)
  jinn@Liu:~$ git clone -b v0.9.1 https://github.com/commaai/openpilot
  jinn@Liu:~$ du -hsc /home/jinn/openpilot
    2.5G	/home/jinn/openpilot
--- move /openpilotX/tools to /openpilot/..
  (sconsvenv) jinn@Liu:~/openpilot$ scons -u -j$(nproc)
--- OK.
  jinn@Liu:~$ du -hsc /home/jinn/openpilot
    3.2G	/home/jinn/openpilot

230424: Trim "DP091aL" to "DP2304PC = DP2304C2"
--- Copy DP091aL/folders to DP2304PC/.
--- remove __pycache__ folders
  jinn@Liu:~/DP2304PC$ find . -type d -name  "__pycache__" -exec rm -r {} +
  jinn@Liu:~/DP2304PC$ find . -name '*JLL*'
  jinn@Liu:~/DP2304PC$ find . -name '*JLL*' -exec rm -f {} +
  jinn@Liu:~/DP2304PC$ rm -rf ./selfdrive/ui/qt/JLL
  jinn@Liu:~/DP2304PC$ find . -name '*.so'
  jinn@Liu:~/OP/DP091a$ find . -name '*.so'
  jinn@Liu:~/DP2304PC$ find . -name '*.a'
  jinn@Liu:~/OP/DP091a$ find . -name '*.a'
  jinn@Liu:~/DP2304PC$ find . -name '*.o'
  jinn@Liu:~/OP/DP091a$ find . -name '*.o'
  jinn@Liu:~/DP2304PC$ find . -name '*.o' -exec rm -f {} +
  jinn@Liu:~/DP2304PC$ find . -name '*.cc'
--- a lot of cc files
  jinn@Liu:~/OP/DP091a$ find . -name '*.cc'
--- only one cc file
    ./third_party/qrcode/QrCode.cc
  jinn@Liu:~$ du -hsc /home/jinn/OP/DP091a
    430M	/home/jinn/OP/DP091a
  jinn@Liu:~$ du -hsc /home/jinn/DP2304PC
    3.1G	/home/jinn/DP2304PC
--- zip to DP2304PC.zip (2.6G)

230422: NG: C2: Install OP0.8.13 - 0.9.1. OK: DP0.9.1, OP0.8.12
--- do 230420 Ccl C (Ccl = Conclusion)
--- Ccl C. Install OP091 on C2 and save C2/openpilot to openpilotDP091.
--- Error: C2 screen nothing
  /data/data/com.termux/files/continue.sh
  root@localhost:/$ cat comma.sh
  root@localhost:/data/openpilot$ cat launch_chffrplus.sh
--- FTP /openpilot/third_party/mapbox-gl-native-qt/aarch64 to C2
--- reboot
--- NG.
--- Change /OP091/selfdrive/ui/ui from
    export LD_LIBRARY_PATH="/system/lib64:$LD_LIBRARY_PATH"
    to
    export LD_LIBRARY_PATH="/system/lib64:/data/openpilot/third_party/mapbox-gl-native-qt/aarch64:$LD_LIBRARY_PATH"
--- FTP /OP091/selfdrive/ui/ui to C2
--- reboot
--- NG.
  root@localhost:/data$ rm -rf openpilot && git clone -b v0.9.1 https://github.com/commaai/openpilot
  root@localhost:/data/openpilot$ scons -j$(nproc)
--- Error 1
    File "/data/openpilot/SConstruct", line 432, in <module>
    /system/comma/usr/bin/../lib/gcc/arm-none-eabi/4.7.1/../../../../arm-none-eabi/bin/ld: cannot find -lgcc
    collect2: error: ld returned 1 exit status
    scons: *** [body/board/obj/body.elf] Error 1
    scons: building terminated because of errors.
--- Change Line 432 - 439 /data/openpilot/SConstruct
--- FTP /home/jinn/openpilot/body/board/obj/body.elf to /data/../
--- Error 1
--- NG: as 230114, 230111  Only OK: 230118
  root@localhost:/data$ rm -rf openpilot
  root@localhost:/data$ cp openpilotDP091 openpilot

230420: NG. UI: PC+C2+body: "body.cc", "Body Face": /assets/body/awake.gif
--- see 230125
--- C2/DP091/selfdrive/ui/qt does not have body.cc.
--- Copy PC/DP091/selfdrive/ui/qt/body.cc to bodyOP091.cc
--- redo 230304
--- note C2/DP091/selfdrive/ui/ui (377 B) and PC/../ui (127 B)
--- Copy C2/../ui to PC/../ui_DP091C2b
--- rename PC/DP091/selfdrive/ui/_ui to _ui_OP091 (27.9 MB)
--- Copy C2/../_ui (27.6 MB) to PC/../_ui
--- Run C2/../_ui on PC
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
--- Error 1
  ./ui: 5: exec: ./_ui: Exec format error
--- compare /ui/ui and /ui/ui_DP091C2b
--- Copy C2/DP091/third_party/mapbox-gl-native-qt/aarch64 to PC/../aarch64
--- Copy PC/../ui to ui_OP091
--- Change PC/../ui Line 3
    export LD_LIBRARY_PATH="/system/lib64:/home/jinn/openpilot/third_party/mapbox-gl-native-qt/aarch64:$LD_LIBRARY_PATH"
    export LD_LIBRARY_PATH="/system/lib64:/openpilot/third_party/mapbox-gl-native-qt/aarch64:$LD_LIBRARY_PATH"
--- NG.
--- rename C2/DP091/selfdrive/ui/_ui to _ui_DP091 (27.6 MB)
--- Copy PC/../_ui_OP091 to C2/../_ui
--- Run PC/../_ui_OP091 on C2
--- C2: reboot
--- Error 2: C2 screen is "Black"
--- Q for Rick: How to re-build _ui on C2/openpilot == DP091, Where C2/../qt files ???
--- Ccl A. C2/../_ui cannot run on PC and vice versa. (Ccl = Conclusion)
        B. We must build and run _ui solely on C2 or solely on PC.
        C. Install OP091 on C2+body and save C2/openpilot to openpilotDP091.

230419: OK CBQ 1: XJ 1-2: PC+C2+body: "controlsdXJ.py", XJ: Changed by JLL
--- Copy "controlsd.py" to "controlsdDP091.py"
  root@localhost:/data/openpilot$ df -ha
    /dev/block/sda10   24G  4.4G   19G  19% /data
      The files under /dev/block/by-name are symbolic links to disk partitions.
      A Secure Digital (SD) card is a tiny flash memory card designed for high-capacity
        memory and various portable devices, such as car navigation systems.
        SDHC: 2GB to 32GB
--- XJ 1: previous Changes
    Line 60 C2/data/openpilot/selfdrive/manager/process_config.py
      #PythonProcess("updated", "selfdrive.updated", enabled=not PC, onroad=False, offroad=True),
    Line 65 DPbeta2/selfdrive/manager/process_config.py to
      PythonProcess("webjoystick", "tools.joystick.web", onroad=False, callback=notcar),
    Line 5 in /data/openpilot/tools/joystick/joystickd.py to
      #from inputs import get_gamepad
    Line 638 /controls/controlsd.py:
      actuators.accel = 1.5*clip(self.sm['testJoystick'].axes[0], -1, 1)
--- XJ 2: Trim "controlsd.py" to "controlsdXJ.py"
    #REPLAY = "REPLAY" in os.environ
    #SIMULATION = "SIMULATION" in os.environ
    #LaneChangeState = log.LateralPlan.LaneChangeState
    #LaneChangeDirection = log.LateralPlan.LaneChangeDirection
    #def set_initial_state(self):
--- Run body
  root@localhost:/data/openpilot/tools/joystick$ python web.py
--- OK
--- CBQ 1: Why is CB so speedy when ignized, Why vibrates a lot, How to calibrate it,
           Can it go straight, Is XJ 3 NG ??? see also 230417 ???
--- Saved "controlsdXJ2.py"
--- XJ: Trim def state_control(self, CS):
    #if not self.joystick_mode:
    #if lac_log.active and not CS.steeringPressed and self.CP.lateralTuning.which() == 'torque' and not self.joystick_mode:
    #elif lac_log.active and lac_log.saturated:
--- NG
--- Read "controlsd.py"
  /selfdrive/controls/controlsd.py
    actuators.accel = 4.0*clip(self.sm['testJoystick'].axes[0], -1, 1)
      self.sm = messaging.SubMaster([...
--- Q1: Who defines axes ??? Yes, by SubMaster see controlsdJLL.py but how ???
    /cereal/messaging/__init__.py
    class SubMaster:
      def __init__(self, services: List[str], poll: Optional[List[str]] = None,
        /cereal/log.capnp
        struct Joystick {
          axes @0: List(Float32);
          buttons @1: List(Bool);

========== Appendix ==========

SConstruct Versions:
  1. OP0812: SConstruct0812  (12.0 kB)
  2. OP091:  SConstruct091   (12.3 kB)
  3. OP092:  SConstructOP092 (12.3 kB) as of 230427
  4. LP092:  SConstructLP092 (13.7 kB) "legacypilot" by Rick Lan
