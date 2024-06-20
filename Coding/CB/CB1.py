CB 1  Jinn-Liang Liu  2023.1.18 - 3.2  All Rights Reserved. © Copyright 2023

========== OP Coding Notes: Run, Change, Read ==========

"openpilot (OP)", "dragonpilot (DP)", "Comma Two (C2A/C2B)", and "Comma Body (CB)" Notes

OP/DP Versions:
  1. DP091: C2    Version 0.9.1 (2022-12-XX) downloaded on 2023-01-17
  2. OP091: PC    Version 0.9.1 (2022-12-XX) downloaded on 2023-01-19

Abbreviations: CAN, Ctr, Md, Nav, UI
  AR (Autonomous Rolling), CAN (Control Area Network), CBQ (CB Question), Ctr (Control),
  Md (Model), Nav (Navigation), UI (User Interface), XJ (Changed by JLL),

. ~/sconsvenv/bin/activate

========== Summary of Coding Notes ==========

230303: OK UI: PC+C2: Run "uiview.py"
  root@localhost:/data/openpilot$ tmux at
  root@localhost:/data/openpilot/selfdrive/debug$ python uiview.py
--- ctrl + s to pause output
--- OK. C2: green path but "Map Loading" later

230302: OK. NG. ??? UI: PC+C2: "joystickd.py", "Joystick Mode", "testJoystick"
--- Change Line 5 in /data/openpilot/tools/joystick/joystickd.py to
      #from inputs import get_gamepad
--- PC Terminal 1: Joystick on your laptop
  root@localhost:/data/openpilot/cereal/messaging$ ./bridge 10.0.0.4 testJoystick
--- echo -n "1" > /data/params/d/JoystickDebugMode
--- PC Terminal 2 (Keys: W: gas, S: brakes, A: +steering, D: -steering)
  root@localhost:/data/openpilot/tools/joystick$ export ZMQ=1
  root@localhost:/data/openpilot/tools/joystick$ python joystickd.py --keyboard
--- OK. Terminal 2
--- NG. C2: No "Joystick Mode"
--- How to set UI in "Joystick Mode" on C2 ??? Wang: ign = True

230226: ToDo. UI: PC: "Replay" + "Joystick Mode"

230225: OK Wang: SnG: PC+C2+body: "joystickd.py", "Joystick Mode", "testJoystick"
  tools/joystick/README.md =>
  root@localhost:/data/openpilot/tools/joystick$ python joystickd.py --keyboard

230222: OK UI: PC: scons Change Run "_uiJLL", "./replay --demo"
--- /selfdrive/ui/mainJLL.cc (_uiJLL) replaces /ui/main091.cc (_ui091)
--- /openpilot/SConstruct
      # build selfdrive/ui/mainJLL.cc
      SConscript(['selfdrive/ui/SConscript'])
--- /openpilot/selfdrive/ui/SConscript
      qt_env.Program("_uiJLL", qt_src + [asset_obj], LIBS=qt_libs)
  (sconsvenv) jinn@Liu:~/openpilot$ scons
--- rename _ui to _ui091, _uiJLL to _ui
--- run replay with 2 terminals
  (sconsvenv) jinn@Liu:~/openpilot/tools/replay$ ./replay --demo
  (sconsvenv) jinn@Liu:~/openpilot$ ./selfdrive/ui/ui
--- OK.

230214: UI: PC+C2+body: ToDo. "Joystick Mode" on C2
    Change: Rick: HomeWindow::mouseDoubleClickEvent

230213: Ctr 1: Pipe 1-3: PC: Read "controlsd.py", "Speed"
--- Ctr 1: Slow CB "Speed" ("Control") down
--- ToDo Change:
    speed_desired = CC.actuators.accel / 20.  # JLL
    actuators.accel = 4.0*clip(self.sm['testJoystick'].axes[0], -1, 1)
    actuators.steer, actuators.steeringAngleDeg = steer, steer * 45.
--- Pipe 1: /car/interfaces.py > CarInterfaceBase(ABC) > /body/interface.py > CarInterface(CarInterfaceBase) >
    /interfaces.py > CarStateBase(ABC) > /body/carstate.py > CarState(CarStateBase) >
    /body/carcontroller.py > CarController > car_helpers.py >
      def load_interfaces(brand_names):
          CarInterface = __import__(.. '.interface'..)
            CarState = __import__(.. '.carstate'..)
          CarController = __import__(.. '.carcontroller'..)
          ret[model_name] = (CarInterface, CarController, CarState)
      interfaces = load_interfaces(interface_names)
      def fingerprint(logcan, sendcan, num_pandas):
        fixed_fingerprint = os.environ.get('FINGERPRINT', "")
          exact_fw_match, fw_candidates = match_fw_to_car(car_fw)
        if len(fw_candidates) == 1:
          car_fingerprint = list(fw_candidates)[0]
        if fixed_fingerprint:
          car_fingerprint = fixed_fingerprint
        return car_fingerprint, finger, vin, car_fw, source, exact_match
      def get_car()
        candidate,,,,, = fingerprint(logcan, sendcan, num_pandas)
        CarInterface, CarController, CarState = interfaces[candidate]
        CP = CarInterface.get_params()
        return CarInterface(CP, CarController, CarState), CP
--- Pipe 2: controlsd.py > main() > Controls() > self.CI, self.CP = get_car() >
    self.CC = car.CarControl > controlsd_thread() > step():
      data_sample() > update_events(CS) > state_transition(CS) >
      state_control(CS) > CC > publish_logs(CS, , CC, lac_log)
--- Pipe 3: /body/carcontroller.py > CarController > def update(,CC,,): return new_actuators, can_sends >
    /body/interface.py > CarInterface > def apply(,c,): return self.CC.update(c,,) >
    controlsd.py > publish_logs() > self.last_actuators, can_sends = self.CI.apply(CC,)

230213a: NG. C2+body: Change "Speed"
--- Change Line 23 /body/carcontroller.py:
    self.speed_pid = PIDController(0.03, k_i=0.23, rate=1/DT_CTRL)  # JLL NG
--- Change Line 10 /body/values.py:
    PEED_FROM_RPM = 0.004  # JLL NG

230205: OK. PC: "./_qtJLL1"
  (sconsvenv) jinn@Liu:~/openpilot/selfdrive/ui/qt/JLL$ scons
  (sconsvenv) jinn@Liu:~/openpilot/selfdrive/ui/qt/JLL$ ./_qtJLL1

230201: OK. PC: Run Change "controlsdJLL.py"
  (sconsvenv) jinn@Liu:~/openpilot$ python controlsdJLL.py
  (sconsvenv) jinn@Liu:~/openpilot$ python controlsd091JLL.py

230122: UI: PC: Read ./_uiJLL: "Check integrity ..." in alerts_offroad.json
--- Pipe: controlsd.py < alertmanager.py < Params().put() < params.py < params_pyx.pyx <
    params.cc < mainJLL.cc < window.cc < home.cc < offroad_alerts.cc > params.get() <

230119: OK UI: PC: Run Change "execvpJLL.py", "./ui"
  (sconsvenv) jinn@Liu:~/openpilot/selfdrive/ui/qt/JLL$ python execvpJLL.py
  (sconsvenv) jinn@Liu:~/openpilot/selfdrive/ui$ ./_ui
--- Pipe: comma.sh > continue.sh > launch_openpilot.sh > launch_chffrplus.sh >
    launch_env.sh > manager.py > process_config.py > process.py >
    import Process > os.execvp > ./ui

230118: OK Install: DP091 = DPbeta2 on C2+body/car
--- Pipe: C2+PC => C2 Fastboot Mode => jinn@Liu:~/eon-neos$ ./flash.sh
    => Continue to Setup => Connect to WiFi => Custom Software => DPbeta2 (DP091)
    => Install Software => Begin Training => ADD jinnliu
    => FTP DP091 from C2 to PC (backup copy) => DP - Cars: COMMA BODY =>
--- OK. body is walking! It keeps slowly turning around at the same spot without
      moving ahead or backward. =>
  root@localhost:/data/openpilot/tools/joystick$ python web.py
    => body rolls too fast!
--- Change Line 65 DPbeta2/selfdrive/manager/process_config.py
    PythonProcess("webjoystick", "tools.joystick.web", onroad=False, callback=notcar),

===== Details of Coding Notes =====

230303a: UI: PC+C2: Run "uiview.py"
  root@localhost:/data/openpilot/selfdrive/debug$ python uiview.py
--- ctrl + s to pause output
--- OK. C2: green path but "Map Loading" later
    /data/pythonpath/cereal/messaging/__init__.py:258: UserWarning: This message has already been written once. Be very careful that you're not setting Text/Struct/List fields more than once, since that will cause memory leaks (both in memory and in the serialized data). You can disable this warning by calling the `clear_write_flag` method of this object after every write.
      dat = dat.to_bytes()
    platform[0] CL_PLATFORM_NAME: QUALCOMM Snapdragon(TM)
    vendor: QUALCOMM
    platform version: OpenCL 2.0 QUALCOMM build: commit #6ff34ae changeid #I0ac3940325 Date: 09/23/16 Fri Local Branch:  Remote Branch: refs/tags/AU_LINUX_ANDROID_LA.HB.1.3.2.06.00.01.214.261
    profile: FULL_PROFILE
    extensions:
    name :QUALCOMM Adreno(TM)
      The Adreno GPU is the part of Snapdragon that creates perfect graphics with super smooth
      motion (up to 144 frames per second FPS) and in HDR (High Dynamic Range) in over a billion
      shades of color.
    device version :OpenCL 2.0 Adreno(TM) 530
    max work group size :1024
    type = 4 = CL_DEVICE_TYPE_GPU
    IOCTL_KGSL_DRAWCTXT_CREATE: creating context with flags 0x84201ad2
    Thneed::clinit done
    Thneed::load: loading from models/supercombo.thneed
    Thneed::clexec: running 368 queued kernels
    Killing old publisher: controlsState
    Traceback (most recent call last):
      File "uiview.py", line 30, in <module>
        pm.send(s, msgs[s])
      File "/data/pythonpath/cereal/messaging/__init__.py", line 259, in send
        self.sock[s].send(dat)
      File "cereal/messaging/messaging_pyx.pyx", line 149, in cereal.messaging.messaging_pyx.PubSocket.send
    cereal.messaging.messaging_pyx.MultiplePublishersError
    selfdrive/modeld/modeld.cc: models loaded, modeld starting
    Starting listener for: camerad
    selfdrive/modeld/modeld.cc: connected main cam with buffer size: 1526004 (1164 x 874)
    Killing old publisher: visionipc_camerad_0
    Stopping listener for: camerad
  root@localhost:/data/openpilot/selfdrive/debug$ python uiview.py
    Thneed::clexec: running 368 queued kernels
    WARNING: linker: ./_ui: unused DT entry: type 0x1d arg 0x94c9
    selfdrive/modeld/modeld.cc: models loaded, modeld starting
    WARNING: linker: /data/openpilot/third_party/mapbox-gl-native-qt/aarch64/libqmapboxgl.so: unused DT entry: type 0x1d arg 0x5374
    Starting listener for: camerad
    selfdrive/modeld/modeld.cc: connected main cam with buffer size: 1526004 (1164 x 874)
    Thneed::clexec: running 368 queued kernels
    Thneed::stop: recorded 19 commands
    : QPixmap::scaleWidth: Pixmap is a null pixmap

230303: OK UI: PC+C2: Run "uiview.py"
  root@localhost:/data/openpilot$ tmux at
  --- now in bash 0
  --- type ` then d (` d)
      [detached (from session comma)]
  root@localhost:/data/openpilot$ tmux at
  --- kill all *d: type ctrl + c to kill all background demon processes (*d like thermald)
  --- everything is dead
  --- type ` then c (` c) and now in bash 1
  --- type ` c in bash 2
  --- exit bash 2
  root@localhost:/data/openpilot$ exit
  root@localhost:/data/openpilot/selfdrive/debug$ python uiview.py
  --- OK. See output on C2 screen.
  --- type ctrl + c to kill uiview
  --- exit tmux (both bash 0 and 1)
  root@localhost:/data/openpilot$ tmux kill-session -t comma
  root@localhost:/data/openpilot$ tmux at
     no sessions
  root@localhost:/data/openpilot$ reboot
  root@localhost:/data/openpilot$ ps au
     PID   USER     TIME  COMMAND
     1455 system    0:00 {ndroid.settings} com.android.settings
     1741 radio     0:00 {.qcrilmsgtunnel} com.qualcomm.qcrilmsgtunnel
     1837 root      0:00 {/data/data/com.} /data/data/com.termux/files/usr/bin/tmux
     1838 root      0:00 {launch_chffrplu} /usr/bin/bash /data/openpilot/launch_chf
     1995 root      0:01 python3 ./manager.py
     2034 root      0:06 ./_ui
     2043 root      0:01 python -m selfdrive.athena.manage_athenad
     2044 root      0:05 ./_sensord
     2045 root      0:01 ./_soundd
     2046 root      0:00 selfdrive.boardd.pandad
     2047 root      0:01 selfdrive.thermald.thermald
     2049 root      0:00 selfdrive.updated
     2052 root      0:00 system.hardware.eon.androidd
     2056 root      0:00 selfdrive.dragonpilot.systemd
     2061 root      0:00 selfdrive.dragonpilot.otisserv
     2177 root      0:01 selfdrive.athena.athenad
     2299 system    0:00 {omm.timeservice} com.qualcomm.timeservice
     2404 root      0:00 /data/data/com.termux/files/usr/bin/sshd -D -R
     2405 root      0:00 /system/comma/usr/bin/bash -l
     2423 root      0:00 ps au
  root@localhost:/data/openpilot$ tmux at
    sensord ui soundd pandad thermald updated androidd systemd otisserv
  root@localhost:/data/openpilot/selfdrive/debug$ python uiview.py
    selfdrive/modeld/modeld.cc: vipc_client_main no frame
    Killing old publisher: visionipc_camerad_0
    Killing old publisher: visionipc_camerad_3
    Killing old publisher: roadCameraState
    Killing old publisher: visionipc_camerad_4
    Killing old publisher: driverCameraState
  root@localhost:/data/openpilot/selfdrive/debug$ ps au
     4849 root      0:00 {tmux: client} tmux at
     5371 root      0:00 ./clocksd
     5373 root      0:02 ./_modeld
     5377 root      0:00 ./ubloxd
     5379 root      0:03 ./locationd
     5388 root      0:01 selfdrive.locationd.calibrationd
     5394 root      0:00 selfdrive.locationd.torqued
     5395 root      0:00 selfdrive.controls.controlsd
     5397 root      0:00 selfdrive.monitoring.dmonitoringd
     5401 root      0:00 selfdrive.navd.navd
     5402 root      0:00 selfdrive.locationd.paramsd
     5407 root      3:25 selfdrive.rtshield
     5408 root      0:02 system.hardware.eon.shutdownd
     5409 root      0:00 selfdrive.dragonpilot.dpmonitoringd
     5411 root      0:05 selfdrive.mapd.mapd
     5412 root      0:04 selfdrive.dragonpilot.gpxd
  --- in bash 0 type ctrl + s to pause output, ctrl + q to resume
    camerad clocksd modeld sensord ubloxd ui soundd locationd calibrationd torqued
    controlsd dmonitoringd navd pandad paramsd plannerd radard thermald rtshield
    shutdownd androidd dpmonitoringd mapd systemd gpxd otisserv
    selfdrive/modeld/modeld.cc: vipc_client_main no frame
  --- OK. C2: green path but "Map Loading" later

230228: NG. UI: PC: Replay: "Joystick Mode"
  (sconsvenv) jinn@Liu:~/openpilot$ ./selfdrive/ui/ui
  (sconsvenv) jinn@Liu:~/openpilot/tools/replay$ ./replay --demo
  (sconsvenv) jinn@Liu:~/openpilot/tools/joystick$ python joystickd.py --keyboard
    Traceback (most recent call last):
      File "joystickd.py", line 7, in <module>
        import cereal.messaging as messaging
    ModuleNotFoundError: No module named 'cereal'

  root@localhost:/data/openpilot/tools/joystick$ python joystickd.py --keyboard
    Traceback (most recent call last):
      File "joystickd.py", line 5, in <module>
        from inputs import get_gamepad
    ModuleNotFoundError: No module named 'inputs'

  root@localhost:/data/openpilot/tools/joystick$ client_loop: send disconnect: Broken pipe

230227: NG. PC+C2: tools/joystick/README.md =>
  root@localhost:/data/openpilot$ echo -n "1" > /data/params/d/JoystickDebugMode
  --- PC Terminal 1
  root@localhost:/data/openpilot/tools/joystick$ python web.py
  --- PC: Settings > Connected WiFi > IPv4 Address 10.0.0.5
  --- PC Terminal 2
  root@localhost:/data/openpilot/cereal/messaging$ ./bridge 10.0.0.5 testJoystick
  --- PC Terminal 1
  root@localhost:/data/openpilot/tools/joystick$ python web.py
    10.0.0.5 - - [27/Feb/2023 06:22:01] "GET /control/0/0.48 HTTP/1.1" 500 -
    Killing old publisher: testJoystick
    [2023-02-27 06:22:01,992] ERROR in app: Exception on /control/0/0.48 [GET]
    Traceback (most recent call last):
      File "/data/data/com.termux/files/usr/lib/python3.8/site-packages/flask/app.py", line 2073, in wsgi_app
        response = self.full_dispatch_request()
      File "/data/data/com.termux/files/usr/lib/python3.8/site-packages/flask/app.py", line 1502, in dispatch_request
        return self.ensure_sync(self.view_functions[rule.endpoint])(**req.view_args)
      File "web.py", line 58, in control
        pm.send('testJoystick', dat)
      File "/data/pythonpath/cereal/messaging/__init__.py", line 259, in send
        self.sock[s].send(dat)
      File "cereal/messaging/messaging_pyx.pyx", line 149, in cereal.messaging.messaging_pyx.PubSocket.send
    cereal.messaging.messaging_pyx.MultiplePublishersError
  --- C2 Screen: Nothing. NG.

230225: OK Wang: SnG: PC+C2+body: "joystickd.py", "Joystick Mode", "testJoystick"
    tools/joystick/README.md => tools/joystick/joystickd.py --keyboard
      joystickd.py >
        import threading
        from inputs import get_gamepad
        from common.params import Params
        parser = argparse.ArgumentParser(description='Publishes events from your joystick to control your car.\n' +
                                                     'openpilot must be offroad before starting joysticked.',
        parser.add_argument('--keyboard', action='store_true', help='Use your keyboard instead of a joystick')
        Params().put_bool('JoystickDebugMode', True) >
          params.py > from common.params_pyx import Params >
            params_pyx.pyx >
              cdef extern from "common/params.h":
                cdef cppclass c_Params "Params":
                  int putBool(string, bool) nogil
              cdef class Params:
                cdef c_Params* p
                def put_bool(self, key, bool val):
                  self.p.putBool(k, val)
              params.h >
                class Params {
                  Params(const std::string &path = {});
                  int put(const char *key, const char *val, size_t value_size);
                  inline int putBool(const std::string &key, bool val) {
                    return put(key.c_str(), val ? "1" : "0", 1);
              params.cc >
                std::unordered_map<std::string, uint32_t> keys = {
                  {"JoystickDebugMode", CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_OFF},
                Params::Params(const std::string &path) {
                  prefix = "/" + util::getenv("OPENPILOT_PREFIX", "d");
                  params_path = ensure_params_path(prefix, path);
                Params::put(const char* key, const char* value, size_t value_size) {
                  // Information about safely and atomically writing a file: https://lwn.net/Articles/457667/
        threading.Thread(target=send_thread, args=(joystick,), daemon=True).start() >
          def send_thread(joystick):
            joystick_sock = messaging.pub_sock('testJoystick')
            while 1:
              dat = messaging.new_message('testJoystick')
              if "WEB" in os.environ:
                import requests
                requests.get("http://"+os.environ["WEB"]+":5000/control/%f/%f" % tuple([joystick.axes_values[a] for a in joystick.axes_order][::-1]), timeout=None)
        while True:
          joystick.update() >
            def update(self):
              joystick_event = get_gamepad()[0]
              event = (joystick_event.code, joystick_event.state)
              elif event[0] in self.axes_values:
                self.max_axis_value[event[0]] = max(event[1], self.max_axis_value[event[0]])
                self.min_axis_value[event[0]] = min(event[1], self.min_axis_value[event[0]])
                norm = -interp(event[1], [self.min_axis_value[event[0]], self.max_axis_value[event[0]]], [-1., 1.])
                self.axes_values[event[0]] = norm if abs(norm) > 0.05 else 0.  # center can be noisy, deadzone of 5%

    echo -n "1" > /data/params/d/JoystickDebugMode
      echo [option(s)] [string(s)], -n: do not print the trailing newline.

    controlsd.py >
      from common.params import Params
      class Controls:
        def __init__(self, sm=None, pm=None, can_sock=None, CI=None):
          self.joystick_mode = self.params.get_bool("JoystickDebugMode") or self.CP.notCar

230223: OK. PC: "Replay" _uiJLL == _ui
  (sconsvenv) jinn@Liu:~/openpilot/tools/replay$ ./replay
    Usage: ./replay [options] route
    Mock openpilot components by publishing logged messages.
    Options:
      -h, --help             Displays this help.
      -a, --allow <allow>    whitelist of services to send
      -b, --block <block>    blacklist of services to send
      -c, --cache <n>        cache <n> segments in memory. default is 5
      -s, --start <seconds>  start from <seconds>
      --demo                 use a demo route instead of providing your own
      --data_dir <data_dir>  local directory with routes
      --prefix <prefix>      set OPENPILOT_PREFIX
      --dcam                 load driver camera
      --ecam                 load wide road camera
      --no-loop              stop at the end of the route
      --no-cache             turn off local cache
      --qcam                 load qcamera
      --no-hw-decoder        disable HW video decoding
      --no-vipc              do not output video
    Arguments:
      route                  the drive to replay. find your drives at
                             connect.comma.ai

  (sconsvenv) jinn@Liu:~/openpilot/tools/replay$ ./replay --demo
    services  std::vector(gyroscope, accelerometer, deviceState, can, controlsState,
      pandaStates, radarState, lateralPlan, liveLocationKalman, carEvents, carParams,
      gpsLocation, wideRoadCameraState, modelV2, managerState, testJoystick, ... qRoadEncodeData)
    loading route  "4cf7a6ad03080c90|2021-09-29--13-46-36"
    load route 4cf7a6ad03080c90|2021-09-29--13-46-36 with 11 valid segments

230223: OK. Change UI: PC: "Replay" w/ _uiJLL
    for /selfdrive/ui/mainJLL.cc (_uiJLL) to replace /ui/main091.cc (_ui091)
    /openpilot/SConstruct
      # build selfdrive/ui/mainJLL.cc
      SConscript(['selfdrive/ui/SConscript'])
    /openpilot/selfdrive/ui/SConscript
      qt_env.Program("_uiJLL", qt_src + [asset_obj], LIBS=qt_libs)
  (sconsvenv) jinn@Liu:~/openpilot$ scons
    rename _uiJLL to _ui
  (sconsvenv) jinn@Liu:~/openpilot/tools/replay$ ./replay  --demo
  (sconsvenv) jinn@Liu:~/openpilot$ ./selfdrive/ui/ui
      Error: w/ //initApp(argc, argv);:
        QOpenGLShader::compile(Fragment): 0:3(12): warning: extension
        0:1(13): error: syntax error, unexpected BASIC_TYPE_TOK, expecting LOWP or MEDIUMP or HIGHP
      OK: w/ initApp(argc, argv);

230220: OK. Change UI: PC: _uiJLL
    for /selfdrive/ui/mainJLL.cc (_uiJLL) to replace /ui/main091.cc (_ui091)
    from /openpilot/SConstruct091
         /selfdrive/ui/qt/JLL/SConstruct
  (sconsvenv) jinn@Liu:~/openpilot/$ scons
    #include "selfdrive/ui/qt/qt_window.h"
             ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    1 error generated.
    scons: *** [selfdrive/ui/mainJLL.o] Error 1
    OK: Due to CPPPATH=["#"]
    selfdrive/ui/qt/widgets/prime.cc:12:10: fatal error: 'QrCode.hpp' file not found
    #include <QrCode.hpp>
             ^~~~~~~~~~~~
    SConscript(['common/transformations/SConscript'])
    # build selfdrive/ui/mainJLL.cc
    SConscript(['selfdrive/ui/SConscript'])
    OK: scons: done building targets.

230219: 2C. Do Replay on PC:
-- build _ui
  /selfdrive/ui/mainJLL.cc (_uiJLL) replaces /ui/main091.cc (_ui091)
  /openpilot/SConstruct
    # build selfdrive/ui/mainJLL.cc
    SConscript(['selfdrive/ui/SConscript'])
  /openpilot/selfdrive/ui/SConscript
    qt_env.Program("_uiJLL", qt_src + [asset_obj], LIBS=qt_libs)
  (sconsvenv) jinn@Liu:~/openpilot$ scons
-- rename _uiJLL to _ui
-- run replay with 2 terminals
  (sconsvenv) jinn@Liu:~/openpilot/tools/replay$ ./replay --demo
  (sconsvenv) jinn@Liu:~/openpilot$ ./selfdrive/ui/ui

-- read code
  tools/replay/main.cc >
    QCommandLineParser parser;
    Replay *replay = new Replay(route, allow, block, nullptr, replay_flags, parser.value("data_dir"), &app);
    replay.cc >
      Replay::Replay() : sm(sm_), flags_(flags), QObject(parent) {
        allow_list.insert(cereal::Event::Which::INIT_DATA);
        allow_list.insert(cereal::Event::Which::CAR_PARAMS);
        qDebug() << "services " << s;
        qDebug() << "loading route " << route;
          pm = std::make_unique<PubMaster>(s);
        route_ = std::make_unique<Route>(route, data_dir);
        events_ = std::make_unique<std::vector<Event *>>();
        new_events_ = std::make_unique<std::vector<Event *>>();
    ConsoleUI console_ui(replay) > new terminal with "openpilot replay 0.9.1 "
    replay->start(parser.value("start").toInt()) >
      Replay::start(int seconds) {
        seekTo(route_->identifier().segment_id * 60 + seconds, false) >
          Replay::seekTo(double seconds, bool relative) {
            queueSegment() >
              Replay::queueSegment() {
                startStream(cur_segment.get()) >
                  Replay::startStream(const Segment *cur_segment) {
                    const auto &events = cur_segment->log->events;
                    // get route start time from initData
                    route_start_ts_ = it != events.end() ? (*it)->mono_time : events[0]->mono_time;
                    // write CarParams
                    car_fingerprint_ = (*it)->event.getCarParams().getCarFingerprint();
                    capnp::MallocMessageBuilder builder;
                    builder.setRoot((*it)->event.getCarParams());
                    Params().put("CarParams", (const char *)bytes.begin(), bytes.size());
                    // start camera server
                    camera_server_ = std::make_unique<CameraServer>(camera_size);
                    // start stream thread
                    stream_thread_ = new QThread();
                    QObject::connect(stream_thread_, &QThread::started, [=]() { stream(); });
                    stream_thread_->start(); >
                      Replay::stream() {
                        while (true) {
                          Event cur_event(cur_which, cur_mono_time_);
                          auto eit = std::upper_bound(events_->begin(), events_->end(), &cur_event, Event::lessThan());
                          const Event *evt = (*eit);
                          // migration for pandaState -> pandaStates to keep UI working for old segments
                          MessageBuilder msg;
                          auto ps = msg.initEvent().initPandaStates(1);
                          ps[0].setIgnitionLine(true);
                          ps[0].setPandaType(cereal::PandaState::PandaType::DOS);
                          pm->send(sockets_[cereal::Event::Which::PANDA_STATES], msg);
                          publishMessage(evt);
                          camera_server_->waitForSent();
                          publishFrame(evt);
                          // wait for frame to be sent before unlock.(frameReader may be deleted after unlock)
                          camera_server_->waitForSent();
                          rInfo("reaches the end of route, restart from beginning");
                          QMetaObject::invokeMethod(this, std::bind(&Replay::seekTo, this, 0, false), Qt::QueuedConnection);
                            > replay.h > route.h > logreader.h > class Event {

230213: Ctr 1: Pipe 1-3: PC: Read "controlsd.py", "Speed"
--- Ctr 1: Slow CB "Speed" ("Control") down
--- ToDo Change:
    speed_desired = CC.actuators.accel / 20.  # JLL
    #speed_desired = CC.actuators.accel / 5.
    actuators.accel = 4.0*clip(self.sm['testJoystick'].axes[0], -1, 1)
    actuators.steer, actuators.steeringAngleDeg = steer, steer * 45.
--- Read: controlsd.py
    /selfdrive/controls/controlsd.py
    class Controls:
      def __init__(self, sm=None, pm=None, can_sock=None, CI=None):
        self.pm = messaging.PubMaster(['sendcan', 'controlsState', 'carState',
                                       'carControl', 'carEvents', 'carParams'])
          /cereal/messaging/__init__.py
          class PubMaster:
            def __init__(self, services: List[str]):
              for s in services:
                self.sock[s] = pub_sock(s)
        print("Waiting for CAN messages...")
        get_one_can(self.can_sock)
        |self.CI, self.CP = get_car(self.can_sock, self.pm.sock['sendcan'], experimental_long_allowed, num_pandas)
          /selfdrive/car/car_helpers.py
            |interfaces = load_interfaces(interface_names)
              def load_interfaces(brand_names):
                path = ('selfdrive.car.%s' % brand_name)
                CarInterface = __import__(path + '.interface', fromlist=['CarInterface']).CarInterface
                  /car/body/interface.py
                  class CarInterface(CarInterfaceBase):
                    ret.carName = "body"
                      /car/interfaces.py
                      class CarInterfaceBase(ABC):
                        def __init__(self, CP, CarController, CarState):
                          self.CP = CP
                          self.CS = CarState(CP)
                          self.CC = CarController(self.cp.dbc_name, CP, self.VM)
                CarState = __import__(path + '.carstate', fromlist=['CarState']).CarState
                  /car/body/carstate.py
                  class CarState(CarStateBase):
                    ret = car.CarState.new_message()
                      /car/interfaces.py
                      class CarStateBase(ABC):
                CarController = __import__(path + '.carcontroller', fromlist=['CarController']).CarController
                  /body/carcontroller.py
                  class CarController:
                    def __init__(self, dbc_name, CP, VM):
                      self.packer = CANPacker(dbc_name)
                      self.speed_pid = PIDController(0.115, k_i=0.23, rate=1/DT_CTRL)
                ret[model_name] = (CarInterface, CarController, CarState)
                return ret
            |def get_car(logcan, sendcan, experimental_long_allowed, num_pandas=1):
              CarInterface, CarController, CarState = interfaces[candidate]
              CP = CarInterface.get_params(candidate, fingerprints, car_fw, experimental_long_allowed)
              return CarInterface(CP, CarController, CarState), CP
        |self.joystick_mode = self.params.get_bool("JoystickDebugMode") or self.CP.notCar
        self.CC = car.CarControl.new_message()  # =! self.CC = CarController
        self.CS_prev = car.CarState.new_message()
        self.LoC = LongControl(self.CP)
        self.LaC: LatControl
        if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
          self.LaC = LatControlAngle(self.CP, self.CI)
              ret.steerControlType = car.CarParams.SteerControlType.angle
        self.last_actuators = car.CarControl.Actuators.new_message()
        if not car_recognized:
        elif self.joystick_mode:
          self.events.add(EventName.joystickDebug, static=True)
        # controlsd is driven by can recv, expected at 100Hz
        self.rk = Ratekeeper(100, print_delay_threshold=None)
    def main(sm=None, pm=None, logcan=None):
      controls = Controls(sm, pm, logcan)
      controls.controlsd_thread()
        def controlsd_thread(self):
          while True:
            self.step()
              def step(self):
                # Sample data from sockets and get a carState
                CS = self.data_sample()
                  def data_sample(self):
                    # Receive data from sockets and update carState
                    # Update carState from CAN
                    can_strs = messaging.drain_sock_raw(self.can_sock, wait_for_one=True)
                    CS = self.CI.update(self.CC, can_strs)
                      /selfdrive/car/interfaces.py
                      def update(self, c: car.CarControl, can_strings: List[bytes]) -> car.CarState:
                    self.distance_traveled += CS.vEgo * DT_CTRL
                    return CS
                # Update control state
                self.state_transition(CS)
                # Compute actuators (runs PID loops and lateral MPC)
                CC, lac_log = self.state_control(CS)
                  def state_control(self, CS):
                    # Given the state, this function returns a CarControl packet
                    lat_plan = self.sm['lateralPlan']
                    long_plan = self.sm['longitudinalPlan']
                    CC = car.CarControl.new_message()
                    CC.latActive = self.active and not CS.steerFaultTemporary and not CS.steerFaultPermanent and \
                                   (not standstill or self.joystick_mode)
                    CC.longActive = self.enabled and not self.events.any(ET.OVERRIDE_LONGITUDINAL) and self.CP.openpilotLongitudinalControl
                    actuators = CC.actuators
                    actuators.longControlState = self.LoC.long_control_state
                    if not self.joystick_mode:
                    else:
                      lac_log = log.ControlsState.LateralDebugState.new_message()
                      if self.sm.rcv_frame['testJoystick'] > 0:
                          actuators.accel = 4.0*clip(self.sm['testJoystick'].axes[0], -1, 1)
                          steer = clip(self.sm['testJoystick'].axes[1], -1, 1)
                          # max angle is 45 for angle-based cars
                          actuators.steer, actuators.steeringAngleDeg = steer, steer * 45.
                        lac_log.active = self.active
                        lac_log.steeringAngleDeg = CS.steeringAngleDeg
                        lac_log.output = actuators.steer
                        lac_log.saturated = abs(actuators.steer) >= 0.9
                    return CC, lac_log
                # Publish data
                self.publish_logs(CS, start_time, CC, lac_log)
                self.CS_prev = CS
    def publish_logs(self, CS, start_time, CC, lac_log):
      # Send actuators and hud commands to the car, send controlsstate and MPC logging
      if self.joystick_mode and self.sm.rcv_frame['testJoystick'] > 0 and self.sm['testJoystick'].buttons[0]:
        CC.cruiseControl.cancel = True
      speeds = self.sm['longitudinalPlan'].speeds
      model_v2 = self.sm['modelV2']
        lane_lines = model_v2.laneLines
        # send car controls over can
        self.last_actuators, can_sends = self.CI.apply(CC, now_nanos)
        self.pm.send('sendcan', can_list_to_can_capnp(can_sends, msgtype='sendcan', valid=CS.canValid))
        CC.actuatorsOutput = self.last_actuators
        if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
          self.steer_limited = abs(CC.actuators.steeringAngleDeg - CC.actuatorsOutput.steeringAngleDeg) > \
                               STEER_ANGLE_SATURATION_THRESHOLD
      # Curvature & Steering angle
      lp = self.sm['liveParameters']
      steer_angle_without_offset = math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg)
      # controlsState
      dat = messaging.new_message('controlsState')
      controlsState = dat.controlsState
      controlsState.state = self.state
      controlsState.longControlState = self.LoC.long_control_state
      if self.joystick_mode:
        controlsState.lateralControlState.debugState = lac_log
      self.pm.send('controlsState', dat)
      # carState
      car_events = self.events.to_msg()
      cs_send = messaging.new_message('carState')
      cs_send.carState = CS
      cs_send.carState.events = car_events
      self.pm.send('carState', cs_send)
      # carParams - logged every 50 seconds (> 1 per segment)
      if (self.sm.frame % int(50. / DT_CTRL) == 0):
        cp_send = messaging.new_message('carParams')
        cp_send.carParams = self.CP
        self.pm.send('carParams', cp_send)
      # carControl
      cc_send = messaging.new_message('carControl')
      cc_send.carControl = CC
      self.pm.send('carControl', cc_send)
      # copy CarControl to pass to CarInterface on the next iteration
      self.CC = CC
        /cereal/messaging/__init__.py
        class PubMaster:
          def send(self, s: str, dat: Union[bytes, capnp.lib.capnp._DynamicStructBuilder]) -> None:
              dat = dat.to_bytes()
            self.sock[s].send(dat)
--- Pipe 1: /car/interfaces.py > CarInterfaceBase(ABC) > /body/interface.py > CarInterface(CarInterfaceBase) >
    /interfaces.py > CarStateBase(ABC) > /body/carstate.py > CarState(CarStateBase) >
    /body/carcontroller.py > CarController > car_helpers.py >
      def load_interfaces(brand_names):
        ret = {}
        for brand_name in brand_names:
          CarInterface = __import__(.. '.interface'..)
            CarState = __import__(.. '.carstate'..)
          CarController = __import__(.. '.carcontroller'..)
        for model_name in brand_names[brand_name]:
          ret[model_name] = (CarInterface, CarController, CarState)
      return ret
      interfaces = load_interfaces(interface_names)
      def get_car()
        CarInterface, CarController, CarState = interfaces[candidate]
        CP = CarInterface.get_params()
        return CarInterface(CP, CarController, CarState), CP
--- Pipe 2: controlsd.py > main() > Controls() > self.CI, self.CP = get_car() >
    self.CC = car.CarControl > controlsd_thread() > step():
      data_sample() > update_events(CS) > state_transition(CS) >
      state_control(CS) > CC > publish_logs(CS, , CC, lac_log)
--- Pipe 3: /body/carcontroller.py > CarController > def update(,CC,,): return new_actuators, can_sends >
    /body/interface.py > CarInterface > def apply(,c,): return self.CC.update(c,,) >
    controlsd.py > publish_logs() > self.last_actuators, can_sends = self.CI.apply(CC,)
--- Note 1: lat_plan and long_plan (not used by body)
    2: LaC = LatControlAngle(CP, CI) (not used by body)
    3: actuators.longControlState = self.LoC.long_control_state
         longControlState @5: LongControlState;
           enum LongControlState {off @0; pid @1; stopping @2; starting @3;}
--- Q1: how to create a qr code for wifi ???
    Q2: who calls controlsState, hudCntrol ???

230213b: Qs for Rick:
  How do you design DP UI?
  What are the coding steps for doing this?
  How do you replace OP UI with DP UI from without to with lane lines?
  Do you also change cc or c code?
  What are cc or c files?
  controlsState uses lac_log. who calls controlsState?

230201: OK. PC: controlsdJLL.py
  (sconsvenv) jinn@Liu:~/openpilot$ python controlsdJLL.py
--- Qt for Beginners:
  (sconsvenv) jinn@Liu:~/openpilot/selfdrive/ui/qt$ g++ qtJLL1.cc -o qtJLL1
--- Error: QApplication: No such file or directory
--- Q1: How to run qtJLL1.cc (QApplication, qt modules)? By SConstruct and SConscript?
  (sconsvenv) jinn@Liu:~/openpilot/selfdrive/ui$ ./_ui
--- Yes. OK.
  (sconsvenv) jinn@Liu:~/openpilot$ python controlsd091JLL.py
--- notes
    ev = Events()
    print(ev.e)
    ev2 = joystick_alert()
    print(ev2)
    joystick_mode = True
    if joystick_mode:
      events.add(EventName.joystickDebug)

230127: 2B. "Joystick Mode": NormalPermanentAlert("Joystick Mode", vals)
  manager.py > manager_thread() > ensure_running(managed_processes.values() >
    managed_processes > PythonProcess("controlsd" >
      controlsd.py > main() > controlsd_thread() > step() >
        get_one_can() > recv_one_retry()
        get_car() > CP > CP.notCar > joystick_mode >
          events.add(EventName.joystickDebug >
            ET.WARNING: joystick_alert > NormalPermanentAlert(_("Joystick Mode"), vals)
            ET.PERMANENT: NormalPermanentAlert(_("Joystick Mode"))
-- notes
  ? Who runs joystick_alert()
  A list(evs)[0]()  # calls joystick_alert() in lib.eventsJLL.
  ? Who calls lib.events in DP091?
  "Body Face": QLabel() in /selfdrive/ui/qt/body.cc
  -- ToDo:
  selfdrive/controls/tests/test_alerts.py
  lib.events: test_state_machine.py, cycle_alerts.py

230125: 2A. "Body Face": /assets/body/awake.gif
--- Problem 2 UI: "Joystick Mode" replaces "Body Face"
  manager.py > main() > managed_processes['ui'].start() > ./_ui
    ui/main.cc > QApplication a >
      MainWindow w >
        window.cc > MainWindow::MainWindow >
          homeWindow = new HomeWindow >
            home.cc > HomeWindow::HomeWindow >
              QHBoxLayout *main_layout = new QHBoxLayout(this);
              slayout = new QStackedLayout();
              main_layout->addLayout(slayout);
              body = new BodyWindow(this); >
                body.cc > BodyWindow::BodyWindow >
                  QStackedLayout *layout = new QStackedLayout(this);
                  face = new QLabel > layout->addWidget(face)
                  awake = new QMovie("../assets/body/awake.gif")
                > BodyWindow::updateState
                  face->setMovie(m);
                  face->movie()->start();
              slayout->addWidget(body)
          main_layout->addWidget(homeWindow)
      setMainWindow(&w) >
        qt_window.cc > setMainWindow(QWidget *w) > w->show()
      > a.exec()
--- Pipe: ui/main.cc: MainWindow w > window.cc: homeWindow >
    home.cc: main_layout > slayout > main_layout->addLayout(slayout) > body >
      body.cc: layout > face > layout->addWidget(face)
    slayout->addWidget(body) > main_layout->addWidget(homeWindow)

230122: 1C: PC output by ./_uiJLL: "Check integrity ..." in alerts_offroad.json
  controlsd.py >
    self.events.add(EventName.carUnrecognized, static=True)
    set_offroad_alert("Offroad_NoFirmware", True)
    def publish_logs()
        car_events = self.events.to_msg()
        cs_send = messaging.new_message('carState')
        cs_send.carState = CS
        cs_send.carState.events = car_events
        self.pm.send('carState', cs_send)
      alertmanager.py >
        with open(os.path.join(BASEDIR, "selfdrive/controls/lib/alerts_offroad.json")) as f:
          OFFROAD_ALERTS = json.load(f)
        def set_offroad_alert(alert: str, show_alert: bool)
          a = OFFROAD_ALERTS[alert]
          Params().put(alert, json.dumps(a))
            params.py > from common.params_pyx import Params >
              params_pyx.pyx >
                cdef cppclass c_Params "Params":
                  int put(string, string) nogil  # GIL (global interpreter lock)
                params.h
                  CLEAR_ON_MANAGER_START = 0x04,
                  CLEAR_ON_IGNITION_ON = 0x08,
                  inline int put(&key, &val) {
                    return put(key, val.data(), val.size());}
                params.cc >
                  std::unordered_map<std::string, uint32_t> keys = {
                    {"OffroPipead_NoFirmware", CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_ON},
                  Params::Params(&path) > int Params::put(key,,)  // Ensuring data reaches disk
                    bytes_written = HANDLE_EINTR(write(tmp_fd, value, value_size))
                      // EINTR means "This call did not succeed (error) because it was interrupted.
                      #include <unistd.h>
                      ssize_t write(int fd, const void *buf, size_t count);
                        // write() writes up to count bytes from the buffer starting at buf
                        // to the file referred to by the file descriptor fd.
  ./_uiJLL > "1 ALERT", "openpilot was unable ... Check integrity ...", "Close" >
    mainJLL.cc > QApplication a >
      MainWindow w >
        window.cc > MainWindow::MainWindow >
          main_layout = new QStackedLayout(this);
          homeWindow = new HomeWindow >
            home.h > class OffroadHome : QStackedLayout* center_layout; >
            home.cc > HomeWindow::HomeWindow >
              QHBoxLayout *main_layout = new QHBoxLayout(this);
              slayout = new QStackedLayout();
              main_layout->addLayout(slayout);
              home = new OffroadHome(this);
                OffroadHome::OffroadHome()
                  QVBoxLayout* main_layout = new QVBoxLayout(this);
                  alert_notif = new QPushButton();
                  alert_notif->setVisible(false);
                  QObject::connect(alert_notif, &QPushButton::clicked, [=] { center_layout->setCurrentIndex(2); });
                  center_layout = new QStackedLayout();
                  alerts_widget = new OffroadAlert();
                  QObject::connect(alerts_widget, &OffroadAlert::dismiss, [=]() { center_layout->setCurrentIndex(0); });
                  center_layout->addWidget(alerts_widget);
                  main_layout->addLayout(center_layout, 1);
                  timer = new QTimer(this);
                  timer->callOnTimeout(this, &OffroadHome::refresh);
                    OffroadHome::refresh() {
                      alerts = alerts_widget->refresh(); >
                        offroad_alerts.h > Params params; >
                        offroad_alerts.cc >
                          AbstractAlert::AbstractAlert
                            QPushButton *dismiss_btn = new QPushButton(tr("Close"));
                            dismiss_btn->setFixedSize(400, 125);
                            QObject::connect(dismiss_btn, &QPushButton::clicked, this, &AbstractAlert::dismiss);
                              > "Close"
                          OffroadAlert::refresh() >
                            QString json = util::read_file("../controls/lib/alerts_offroad.json").c_str();
                            std::string bytes = params.get(key);  // from controlsd.py ? Yes
                              //---JLL {"text": "openpilot was unable ... Check integrity ...", "severity": 0}
                              //printf("//---JLL %s\n", bytes.c_str());
                            if (bytes.size()) {
                              auto doc_par = QJsonDocument::fromJson(bytes.c_str());
                              text = doc_par["text"].toString();
                            label->setVisible(!text.isEmpty());  // JLL // > text is gone
                               > "Check integrity ..."
                      // pop-up new notification
                      alert_notif->setVisible(alerts);
                      if (alerts) {
                        alert_notif->setText(QString::number(alerts) + (alerts > 1 ? tr(" ALERTS") : tr(" ALERT")));
                          > 1 > "1 ALERT"
              QObject::connect(home, &OffroadHome::openSettings, this, &HomeWindow::openSettings);
              slayout->addWidget(home);
          main_layout->addWidget(homeWindow)
      setMainWindow(&w) >
        qt_window.cc > setMainWindow(QWidget *w) > w->show()
      > a.exec()
--- Pipe: controlsd.py < alertmanager.py < Params().put() < params.py < params_pyx.pyx <
    params.cc < mainJLL.cc < window.cc < home.cc < offroad_alerts.cc > params.get() <
--- ToDo
  ALL TIME > /drive_stats.cc > add_stats_layouts(tr("ALL TIME"), all_) >
  CHILL MODE ON > ExperimentalModeButton::showEvent
  class Alert for py files
  struct Alert for C++ files
  printf("//---JLL %s\n", bytes.c_str());

-- sound.cc does not call ui.cc
    # build main UI
  qt_src = ["main.cc", "qt/sidebar.cc", "qt/onroad.cc", "qt/body.cc", ...
    # build soundd
  qt_env.Program("soundd/_soundd", ["soundd/main.cc", "soundd/sound.cc"], LIBS=qt_libs)

  (sconsvenv) jinn@Liu:~/openpilot/selfdrive/ui/soundd$ ./_soundd

  ui/SConscript >
    widgets_src = ["ui.cc",
    widgets = qt_env.Library("qt_widgets", widgets_src, LIBS=base_libs)
    qt_libs = [widgets, qt_util] + base_libs
    qt_src = ["main.cc", "qt/sidebar.cc", "qt/onroad.cc", "qt/body.cc",
    qt_env.Program("_ui", qt_src + [asset_obj], LIBS=qt_libs) > _ui

-- env.Library vs env.SharedLibrary
  Differences between static and dynamic libraries
    Static libraries (by env.Library: z.c > z.o > lib.a > a.out), while reusable
      in multiple programs, are locked into a program at compile time.
    Dynamic or shared libraries (env.SharedLibrary: z.c > z.o > lib.so > a.out)
      exist as separate files outside of the executable file at runtime.
      If a second program linked with the same shared library is executed,
      it can use the same copy of the shared library, thus saving a lot of memory.

  controlsd.py >
    self.AM = AlertManager()
    self.events = Events()
    alerts = self.events.create_alerts
    self.AM.add_many(self.sm.frame, alerts)
    current_alert = self.AM.process_alerts(self.sm.frame, clear_event_types)

230121: 1B: CB output on C2: Waiting for controls to start
  NativeProcess("soundd", "selfdrive/ui/soundd", ["./soundd"], offroad=True),
  ui/soundd/sound.cc >
    Sound::update() > setAlert(Alert::get(sm, started_frame)) > ui.h >
      struct Alert {
        QString text1; QString text2; QString type;
        cereal::ControlsState::AlertSize size;
        AudibleAlert sound;
        static Alert get(const SubMaster &sm, uint64_t started_frame) {
          if (sm.rcv_frame("controlsState") < started_frame) {
            // car is started, but controlsState hasn't been seen at all
            return {"openpilot Unavailable", "Waiting for controls to start",
                    "controlsWaiting", cereal::ControlsState::AlertSize::MID,
                    AudibleAlert::NONE};
    Sound::setAlert(const Alert &alert)

230119: OK UI: PC: Run Change "execvpJLL.py", "./ui"
--- 1A: CB: What are the code steps that yield the Error message?
  comma.sh > continue.sh > launch_openpilot.sh > launch_chffrplus.sh > launch_env.sh >
    manager.py > managed_processes['ui'].start()
      process_config.py >
        procs = [ ..., NativeProcess("ui", "selfdrive/ui", ["./ui"],,,), ...] >
        managed_processes = {p.name: p for p in procs}
          process.py >
            from multiprocessing import Process
              # multiprocessing — Process-based parallelism
            class ManagerProcess(ABC): proc: Optional[Process] = None, start(self) >
            class NativeProcess(ManagerProcess): self.name = name, self.cwd = cwd, self.cmdline = cmdline
              start(self):
                cwd = os.path.join(BASEDIR, self.cwd),
                self.proc = Process(name=self.name, target=nativelauncher, args=(self.cmdline, cwd, self.name)),
                  def nativelauncher(pargs: List[str], cwd: str, name: str) -> None:
                      # exec the process
                    os.chdir(cwd)
                    os.execvp(pargs[0], pargs)
                      - pargs = self.cmdline = cmdline = ["./ui"], pargs[0] = "./ui"
                      - “execvp” system call in Python: Everything you need to know!
                      - “v” means the number of parameters is variable, with the arguments
                          being passed in a list or tuple as the args parameter.
                      - “p” uses the PATH environment variable to locate the program file.
                      - command = ["python", "hello.py"] > os.execvp(command[0], command)
                self.proc.start() > ./ui
  (sconsvenv) jinn@Liu:~/openpilot/selfdrive/ui/qt/JLL$ python execvpJLL.py
  (sconsvenv) jinn@Liu:~/openpilot/selfdrive/ui$ ./_ui
--- Pipe: comma.sh > continue.sh > launch_openpilot.sh > launch_chffrplus.sh >
    launch_env.sh > manager.py > process_config.py > process.py >
    import Process > os.execvp > ./ui

230118: OK Install: DP091 = DPbeta2 on C2+body/car
    C2+PC => C2 Fastboot Mode => jinn@Liu:~/eon-neos$ ./flash.sh
    => Continue to Setup => Connect to WiFi => Custom Software => DPbeta2
    => Install Software => Begin Training => ADD jinnliu
    => FTP DPbeta2 (0.9.1) from C2 to PC (backup copy)
    => DP - Cars: COMMA BODY =>
    OK. body is walking! It keeps slowly turning around at the same spot without
        moving ahead or backward. =>
    root@localhost:/data/openpilot/tools/joystick$ python web.py
    => Error: “openpilot Unavailable Waiting for controls to start” (应该是panda没连接好)
    => Error: same => Joystick => root@localhost:/data/openpilot/tools/joystick$ python web.py =>
       http://10.0.0.6:5000/ => Error: same =>
    => Panda was NOT properly connected. Adjusted it.
    OK. body is walking! It keeps slowly turning around at the same spot without moving ahead or backward.  =>
    root@localhost:/data/openpilot/tools/joystick$ python web.py
     * Serving Flask app 'web' (lazy loading)
    ...
      File "web.py", line 78, in <module>
    OSError: [Errno 98] Address already in use

    Change: DPbeta2/selfdrive/manager/process_config.py
      Line 65 Old:
        # PythonProcess("webjoystick", "tools.joystick.web", onroad=False, callback=notcar),
      Line 65 New:
        PythonProcess("webjoystick", "tools.joystick.web", onroad=False, callback=notcar),
--- In:  Press Ig Button
--- Pipe Run: launch_openpilot.sh => launch_chffrplus.sh => launch_env.sh =>
              ./_ui => ./manager.py => PythonProcess("controlsd",)
--- Out: "Body Face": assets/body/awake.gif
         "Joystick Mode": NormalPermanentAlert("Joystick Mode", vals)
--- Problems:
      1. Error: “openpilot Unavailable Waiting for controls to start”
      2. UI: Joystick Mode replaces "Body Face"
      3. Speed: test_cruise_speed.py

--- Problem 1 Error: “openpilot Unavailable Waiting for controls to start”
--- OK. This problem was solved due to panda borad was not properly inserted onto
    the main borad of comma body.

========== Appendix ==========

----- Tutorial

The Independent Qt® Tutorial
  Q_OBJECT macro does some wonderful magic such as enabling signals and slots.
  QtConcurrent framework has higher level of functional type APIs and we can
    even cancel, pause, or resume the results from a thread run.
Docker — A Beginner's guide to Dockerfile with a sample project
  Docker 初探：基本指令與簡單介紹 Dockerfile 和 docker-compose
  Explaining Writing Dockerfile with Examples
Lambda expression in C++
  [&] : capture all external variables by reference
  [=] : capture all external variables by value
  [a, &b] : capture a by value and b by reference
  [ ] can only access variables which are local to it.
Ellipsis in C++ with Examples
  Ellipsis: ...
  Ellipsis in C++ allows the function to accept an indeterminate number of arguments.
  logMessage(ReplyMsgType type, const char *fmt, ...)
  va_list, va_start, va_end
  va_start is defined in <stdarg.h> but not found in tools/replay/util.cc.
    Why? OK. See C++ 14
  https://dotnettutorials.net/lesson/ellipsis-in-cpp/
