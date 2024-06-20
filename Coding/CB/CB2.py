CB 2  Jinn-Liang Liu  2023.3.3 - 3.23  All Rights Reserved. © Copyright 2023

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

230323: OK UI: C2: Run "process_config.py"
  Change Line 60 C2/data/openpilot/selfdrive/manager/process_config.py
    #PythonProcess("updated", "selfdrive.updated", enabled=not PC, onroad=False, offroad=True),
  root@localhost:/data/openpilot$ tmux at

230322: UI: C2: Read for Alert: "Connect to internet to check for updates"
  root@localhost:/data/openpilot$ tmux at
    ui updated controlsd ...
--- Pipe: updated > updated.py > alertmanager.py > Params().put > 230122 Pipe > params.py >
    params_pyx.pyx > params.cc > ui > mainJLL.cc > window.cc > home.cc > offroad_alerts.cc > params.get

230319: ??? UI: PC+C2: Run Read "tmux at"
  root@localhost:/data/openpilot$ tmux at
    camerad clocksd modeld sensord ui controlsd dmonitoringd pandad paramsd plannerd
    radard dpmonitoringd mapd systemd ...  (no "loggerd" ???)
--- Pipe: dp_conf.py > process_config.py > manager.py
  JLL #subprocess.call("./bootlog", cwd=os.path.join(BASEDIR, "selfdrive/loggerd"))
  JLL # {'name': 'dp_logger', 'default': False, 'type': 'Bool', 'conf_type': ['param']},
  JLL # NativeProcess("encoderd", "selfdrive/loggerd", ["./encoderd"]),
  JLL # NativeProcess("loggerd", "selfdrive/loggerd", ["./loggerd"], onroad=False, callback=logging),
--- How to log "fcamera.hevc" in "C2/data/media/0/dashcam" (by "loggerd") ???

230318: OK Run Car+C2: Use "Enable On-Road Dashcam", "啟用行車記錄功能"
--- Output videos
  /data/openpilot/installer/media/0/dashcam/2023-03-18_02-03-21.mp4
  /home/jinn/OP/CBRun/data/openpilot/installer/media/0/dashcam
    /2023-03-18_02-03-21.mp4, /2023-03-18_02-03-21.hevc (converted on-line)
--- ToDo: "loggerd" with "fcamera.hevc" in "/data/media/0/dashcam"

230317: OK NG UI: PC+C2: Run "uiview.py" ("Green Path" on C2. No "fcamera.hevc")
--- ssh to C2 on Terminal 1
  jinn@Liu:~$ ssh 10.0.0.2
  root@localhost:/data/openpilot$ tmux at
--- type ctrl+s to pause output, ctrl+q to resume
--- ssh on Terminal 2
  root@localhost:/data/openpilot/selfdrive/debug$ python uiview.py
--- OK. See "Green Path" on C2 screen and the following on Terminal 2
--- if not OK, type ctrl+c and then python uiview.py
--- OK and NG: C2 doesn't have the folder "/data/media/0/realdata/" or "fcamera.hevc"

230311: OK UI: PC: Change "cameraJLL.cc", scons Change Run "./replayJLL --data_dir"
--- Change "SConscript"
--- scons Run "./replayJLL --data_dir"
--- Errors on Screen
    Error 1: │Error sending a packet for decoding: -1094995529                                                                                                                │
    Error 2: │camera[0] failed to get frame: 1                                                                                                                                │
--- Read "mainJLL.cc"
--- Pipe 1: /replay/mainJLL.cc > replayJLL.cc > routeJLL.cc > cameraJLL.cc >
            framereader.cc > /tools/ubuntu_setup.sh > ffmpeg > <libavcodec/avcodec.h>
--- Pipe 2: main() > Replay::start() > Replay::seekTo() > Replay::queueSegment() > Segment::Segment() >
    Segment::loadFile() > FrameReader::load() > decoder = avcodec_find_decoder() >
    decoder_ctx = avcodec_alloc_context3(decoder) > pkt = av_packet_alloc() >
    packets.push_back(pkt) > QObject::connect() > Replay::startStream() >
    CameraServer::CameraServer() > CameraServer::startVipcServer() >
    CameraServer::cameraThread() >
      FrameReader::get() > yuv_buf = vipc_server_->get_buffer() > FrameReader::get(*yuv) >
      FrameReader::decode() > FrameReader::decodeFrame() > avcodec_send_packet(decoder_ctx, pkt)
--- Read Screen output
--- Pipe 3: /replay/mainJLL.cc > consoleuiJLL.cc
--- Error 2: Debug output on screen
--- Change "cameraJLL.cc"
--- Pipe 3: /replay/mainJLL.cc > consoleuiJLL.cc
--- OK: Error 2 and 230310 NG Solved!

230310: OK. NG. UI: PC: Change "routeJLL.cc", Run "./replayJLL --data_dir"
--- Change "routeJLL.cc"
--- Solved 230307 NG ToDo
--- NG. UI shows but vedio is running then freezes.

230309: OK UI: PC: scons Change Run "./replayJLL --demo"
--- Copy "SConscript" to SConscript091 and Change (230307 ToDo)
--- Change "SConstruct"
--- scons Run "replayJLL"

230307: OK. NG. ToDo. UI: PC: Run "./replay --demo, --data_dir"
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
  (sconsvenv) jinn@Liu:~/openpilot/tools/replay$
    ./replay --demo
--- OK. --demo
--- NG. --data_dir
--- Pipe: /replay/main.cc > /replay/route.cc
--- ToDo: Change /replay/route.cc Route::loadFromLocal

230306: OK UI: PC: Change Run "cycle_alertsJLL.py", "Joystick Mode", "testJoystick"
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python cycle_alertsJLL.py

--- Solved UI 1: "Joystick Mode" on PC

230305: OK. NG. UI: PC: Run "joystickdJLL.py  --keyboard"
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python bodyJLL.py

--- OK.
  (sconsvenv) jinn@Liu:~/openpilot$ python joystickdJLL.py  --keyboard
--- NG.
  Params().put_bool('JoystickDebugMode', True)  # controlsd.py uses it not UI
  UI doesn't sm 'carState', 'carEvents', 'carParams', 'controlsState' for "Joystick Mode"
  UI doesn't use joystickd.py for "Joystick Mode"

230304: OK UI: PC: Run "uiviewJLL.py", "bodyJLL.py"
--- rename _ui to _uiJLL, _ui091 to _ui
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
  (sconsvenv) jinn@Liu:~/openpilot/tools/replay$ ./replay --demo
--- OK. Same as 230223:
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python bodyJLL.py
viewJLL.py
  (sconsvenv) jinn@Liu:~/openpilot$ python bodyJLL.py
--- Solved UI 1: "Body Face" on PC

===== Details of Coding Notes =====

230323: OK UI: C2: Run "process_config.py"
  Change C2: data/openpilot/selfdrive/manager/process_config.py
  Line 60 New:
    #PythonProcess("updated", "selfdrive.updated", enabled=not PC, onroad=False, offroad=True),
  root@localhost:/data/openpilot$ tmux at
    sensord ui soundd pandad thermald updated androidd systemd otisserv
    sensord ui soundd pandad thermald androidd systemd otisserv
--- "otisserv"
  /DP091/selfdrive/dragonpilot/otisserv.py
  "https://api.mapbox.com/geocoding/v5/mapbox.places/"
  "geocoding": Geocoding API: The forward geocoding query type allows you to look
    up a single location by name and returns its geographic coordinates.

230322: UI: C2: Read for Alert: "Connect to internet to check for updates"
  root@localhost:/data/openpilot$ tmux at
    ui updated controlsd ...
  /openpilot/selfdrive/updated.py
    from selfdrive.controls.lib.alertmanager import set_offroad_alert
    class Updater:
    def main() -> None:
      updater = Updater()
      update_failed_count = 0  # TODO: Load from param?
      while True:
        updater.set_params(update_failed_count, exception)
          def set_params(self, failed_count: int, exception: Optional[str]) -> None:
            now = datetime.datetime.utcnow()
            dt = now - last_update
            if failed_count > 15 and exception is not None and self.has_internet:
            elif dt.days > DAYS_NO_CONNECTIVITY_MAX and failed_count > 1:
              set_offroad_alert("Offroad_ConnectivityNeeded", True)
                /openpilot/selfdrive/controls/lib/alertmanager.py
                  from common.params import Params
                  with open(os.path.join(BASEDIR, "selfdrive/controls/lib/alerts_offroad.json")) as f:
                    OFFROAD_ALERTS = json.load(f)
                      /openpilot/selfdrive/controls/lib/alerts_offroad.json
                        "Offroad_ConnectivityNeeded": {
                          "text": "Connect to internet to check for updates. openpilot won't automatically start until it connects to internet to check for updates.",
                  def set_offroad_alert(alert: str, show_alert: bool)
                    a = OFFROAD_ALERTS[alert]
                    Params().put(alert, json.dumps(a)) > see 230122, 230225 >
                    > params.py > params_pyx.pyx > params.cc > Params::put(,,) > write(, , )
        update_failed_count += 1
  ./_uiJLL > "1 ALERT", "Connect to internet ... "
--- Pipe: updated > updated.py > alertmanager.py > Params().put > 230122 Pipe > params.py >
    params_pyx.pyx > params.cc > ui > mainJLL.cc > window.cc > home.cc > offroad_alerts.cc > params.get

230319: ??? UI: PC+C2: Run Read "tmux at"
  root@localhost:/data/openpilot$ tmux at
    camerad clocksd modeld sensord ui controlsd dmonitoringd pandad paramsd plannerd
    radard dpmonitoringd mapd systemd ... (no "loggerd" ???)
--- CAN 26
  (sconsvenv) jinn@Liu:~/openpilot$ python zc.py
    selfdrive/loggerd/bootlog.cc: bootlog to /home/jinn/.comma/media/0/realdata/boot/2023-03-19--19-19-44
--- How to view 2023-03-19--19-19-44 ???
--- Read for "loggerd" in manager.py (see 230119)
    /DP091/selfdrive/manager/manager.py
      from selfdrive.manager.process_config import managed_processes
      from common.dp_conf import init_params_vals
      def main() -> None:
        manager_init()
          def manager_init() -> None:
            JLL #subprocess.call("./bootlog", cwd=os.path.join(BASEDIR, "selfdrive/loggerd"))
            init_params_vals(params)
              /DP091/common/dp_conf.py
                confs = [
                  JLL # {'name': 'dp_logger', 'default': False, 'type': 'Bool', 'conf_type': ['param']},
                def init_params_vals(params):
                  for conf in confs:
                    if 'param' in conf['conf_type']:
        manager_thread()
          def manager_thread() -> None:
            while True:
              ensure_running(managed_processes.values(), started, params=params, CP=sm['carParams'], not_run=ignore)
                /DP091/selfdrive/manager/process_config.py
                  JLL # NativeProcess("encoderd", "selfdrive/loggerd", ["./encoderd"]),
                  JLL # NativeProcess("loggerd", "selfdrive/loggerd", ["./loggerd"], onroad=False, callback=logging),
                  managed_processes = {p.name: p for p in procs}
--- Pipe: dp_conf.py > process_config.py > manager.py
--- ToDo: How to log "fcamera.hevc" in "C2/data/media/0/dashcam" (by "loggerd") ???

230318: OK Run Car+C2: Use "Enable On-Road Dashcam", "啟用行車記錄功能"
--- Output videos
    /data/openpilot/installer/media/0/dashcam/2023-03-18_02-03-21.mp4
    /home/jinn/OP/CBRun/data/openpilot/installer/media/0/dashcam
      /2023-03-18_02-03-21.mp4, /2023-03-18_02-03-21.hevc (converted on-line)
--- Read for "啟用行車記錄功能"
      /DP091/selfdrive/ui/ "no SConscript"
    /openpilot/selfdrive/ui/SConscript091
        # build translation files
      with open(File("translations/languages.json").abspath) as f:
        languages = json.loads(f.read())
          /DP091/selfdrive/ui/translations/languages.json
            "中文（繁體）": "main_zh-CHT",
      translation_sources = [f"#selfdrive/ui/translations/{l}.ts" for l in languages.values()]
        /openpilot/selfdrive/ui/translations/main_zh-CHT.ts > "no 啟用行車記錄功能"
            /DP091/selfdrive/ui/translations/main_zh-CHT.ts
              <context>
                  <name>DPGeneralPanel</name>
                <source>Enable On-Road Dashcam</source>
                <translation>啟用行車記錄功能</translation>
      translation_targets = [src.replace(".ts", ".qm") for src in translation_sources]
        # WHAT IS A .QM FILE? QM stands for Qt message file, in a binary format.
      lrelease_bin = 'third_party/qt5/larch64/bin/lrelease' if arch == 'larch64' else 'lrelease'
      lupdate = qt_env.Command(translation_sources, qt_src, "selfdrive/ui/update_translations.py")
        # Command(target, source, action, [key=val, ...])
        # Executes a specific action (or list of actions) to build a target file or files.
      lrelease = qt_env.Command(translation_targets, translation_sources, f"{lrelease_bin} $SOURCES")
      qt_env.Depends(lrelease, lupdate)
      qt_env.Precious(translation_sources)  # Preventing Removal of Targets: the Precious Function
--- Read for "/data/openpilot/installer/media/0/dashcam"
      /DP091/selfdrive/ui/ "no SConscript"
    /openpilot/selfdrive/ui/SConscript091
      if GetOption('extras'):
          # build installers
        senv = qt_env.Clone()
        installers = [
          ("dashcam", dashcam),
        for brand in ("openpilot", "dashcam"):
          cont[brand] = senv.Command(f"installer/continue_{brand}.o",
            f"installer/continue_{brand}.sh", "ld -r -b binary -o $TARGET $SOURCE")
            /openpilot/selfdrive/ui/installer/continue_dashcam.sh
              cd /data/openpilot
              exec ./launch_chffrplus.sh
                /data/openpilot/launch_chffrplus.sh
                  cd selfdrive/manager
                  ./build.py && ./manager.py
        for name, branch in installers:
          brand = "dashcam" if "dashcam" in branch else "openpilot"
          obj = senv.Object(f"installer/installers/installer_{name}.o", ["installer/installer.cc"], CPPDEFINES=d)
            # obj = env.Object(target ='hello.o', source = 'hello.c')
            # Object	compile or assemble an object file
          f = senv.Program(f"installer/installers/installer_{name}", [obj, cont[brand]], LIBS=qt_libs)
            # Program	link objects and/or libraries into an executable
            # Library	archive files into a library
            # SharedLibrary	archive files into a shared library
            > /ui/installer/installers/installer_dashcam > "not found", not installed
--- Q: who installs "/data/openpilot/installer/media/0/dashcam" ???
    A: /OP091/selfdrive/ui/installer/installer.cc but
       /data/openpilot/selfdrive/ui "no /installer"
--- Read for "/data/media/0/dashcam" which is "empty" but
      "/data/openpilot/installer/media/0/dashcam/2023-03-18_02-03-21.mp4
--- Q: Who recorded "2023-03-18_02-03-21.mp4" ???
    /data/openpilot/selfdrive/manager/manager.py
    from selfdrive.manager.process_config import managed_processes
      /data/openpilot/selfdrive/manager/process_config.py
        PythonProcess("systemd", "selfdrive.dragonpilot.systemd", offroad=True),
          /data/openpilot/selfdrive/dragonpilot/systemd.py
            from selfdrive.dragonpilot.dashcamd import Dashcamd
              /data/openpilot/selfdrive/dragonpilot/dashcamd.py
                DASHCAM_VIDEOS_PATH = '/data/media/0/dashcam/'
                  def make_folder(self):
                    os.makedirs(DASHCAM_VIDEOS_PATH)
                  def start(self):
                      # start recording
                    file_name = now.strftime("%Y-%m-%d_%H-%M-%S")
                    os.system("LD_LIBRARY_PATH= screenrecord --bit-rate %s
                      --time-limit %s %s%s.mp4 &" % (DASHCAM_BIT_RATES,
                      DASHCAM_DURATION, DASHCAM_VIDEOS_PATH, file_name))
--- A. screenrecord recorded "2023-03-18_02-03-21.mp4"
--- Read "os.system", "LD_LIBRARY_PATH", "screenrecord"
    system(const char *command) passes the string pointed to by
      command to the host environment so that a command processor can execute it.
    adb shell screenrecord /mnt/sdcard/Download/test.mp4
    Android screenrecord v1.2. Records the device's display to a .mp4 file
    adb: Android Debug Bridge; adb shell screenrecord --help
    jinn@Liu:~$ adb --help
    jinn@Liu:~$ adb devices
      List of devices attached "none"
    root@localhost:/data/openpilot$ compgen -c
      ... screenrecord
    root@localhost:/data/openpilot$ env | grep '^LD_LIBRARY_PATH'
      LD_LIBRARY_PATH=/data/phonelibs:/data/data/com.termux/files/usr/lib
--- ToDo: "loggerd" with "fcamera.hevc" in "/data/media/0/dashcam"
--- Conclusion:
    /OP091/selfdrive/ui/installer/
    /data/openpilot/selfdrive/ui "no /installer"
    /data/openpilot/installer/media/0/dashcam/2023-03-18_02-03-21.mp4
    /data/media/0/dashcam/ "empty"
    /data/media/0/realdata/
    /data/media/0/fakedata/
    /data/openpilot == /DP091, /PC/openpilot == /OP091
--- Ask Rick Lan for missing files and folders

230317: OK NG UI: PC+C2: Run "uiview.py" ("Green Path" on C2. No "fcamera.hevc")
--- ssh to C2 on Terminal 1
  jinn@Liu:~$ ssh 10.0.0.2
  root@localhost:/data/openpilot$ tmux at
--- type ctrl+s to pause output, ctrl+q to resume
--- ssh again on Terminal 2
  root@localhost:/data/openpilot/selfdrive/debug$ python uiview.py
--- OK. See "Green Path" on C2 screen and the following on Terminal 2
    selfdrive/ui/qt/onroad.cc: slow frame rate: 14.44 fps
    : QPixmap::scaleWidth: Pixmap is a null pixmap
--- if not OK, type ctrl+c and then python uiview.py
    selfdrive/modeld/modeld.cc: vipc_client_main no frame
    child selfdrive.locationd.calibrationd got SIGINT
    Stopping listener for: camerad
--- Error on Terminal 1
    selfdrive/modeld/modeld.cc: vipc_client_main no frame
--- FileZilla to C2 and check /data/media/0/realdata/
--- NG: No /data/media/0/realdata/
--- Q: How to generate /realdata?
--- use hand to move C2
--- OK on Terminal 2
    /data/pythonpath/cereal/messaging/__init__.py:258: UserWarning: This message has already ...
      dat = dat.to_bytes()
    platform[0] CL_PLATFORM_NAME: QUALCOMM Snapdragon(TM)
    vendor: QUALCOMM
    platform version: OpenCL 2.0 QUALCOMM build: commit #6ff34ae changeid #I0ac3940325 Date: 09/23/16 Fri Local Branch:  Remote Branch: refs/tags/AU_LINUX_ANDROID_LA.HB.1.3.2.06.00.01.214.261
    profile: FULL_PROFILE
    extensions:
    name :QUALCOMM Adreno(TM)
    device version :OpenCL 2.0 Adreno(TM) 530
    max work group size :1024
    type = 4 = CL_DEVICE_TYPE_GPU
    WARNING: linker: ./_ui: unused DT entry: type 0x1d arg 0x94c9
    WARNING: linker: /data/openpilot/third_party/mapbox-gl-native-qt/aarch64/libqmapboxgl.so: unused DT entry: type 0x1d arg 0x5374
    IOCTL_KGSL_DRAWCTXT_CREATE: creating context with flags 0x84201ad2
    Thneed::clinit done
    Thneed::load: loading from models/supercombo.thneed
    Starting listener for: camerad
      Thneed::clexec: running 368 queued kernels
    selfdrive/modeld/modeld.cc: models loaded, modeld starting
    selfdrive/modeld/modeld.cc: connected main cam with buffer size: 1526004 (1164 x 874)
      selfdrive/modeld/modeld.cc: skipping model eval. Dropped 4 frames
      Thneed::clexec: running 368 queued kernels
      Thneed::stop: recorded 19 commands
    Killing old publisher: controlsState
    Traceback (most recent call last):
      File "uiview.py", line 30, in <module>
        pm.send(s, msgs[s])
      File "/data/pythonpath/cereal/messaging/__init__.py", line 259, in send
        self.sock[s].send(dat)
      File "cereal/messaging/messaging_pyx.pyx", line 149, in cereal.messaging.messaging_pyx.PubSocket.send
    cereal.messaging.messaging_pyx.MultiplePublishersError
    Killing old publisher: visionipc_camerad_0
    Thneed::clexec: running 368 queued kernels
    Thneed::stop: recorded 19 commands
    Killing old publisher: modelV2
    Killing old publisher: cameraOdometry
    Killing old publisher: visionipc_camerad_3
    selfdrive/modeld/modeld.cc: vipc_client_main no frame
    selfdrive/modeld/modeld.cc: models loaded, modeld starting
    Starting listener for: camerad
    selfdrive/modeld/modeld.cc: connected main cam with buffer size: 1526004 (1164 x 874)
    Thneed::clexec: running 368 queued kernels
    Thneed::stop: recorded 19 commands
    : QPixmap::scaleWidth: Pixmap is a null pixmap
    selfdrive/ui/qt/onroad.cc: slow frame rate: 14.94 fps
    selfdrive/ui/qt/onroad.cc: slow frame rate: 14.85 fps
--- Errors on C2
    "Map Loading"
--- Error on Terminal 2
    : [ ERROR ]  "{unknown}[Setup]: loading style failed: HTTP status code 403"
--- Errors on Terminal 1: Red "camerad", "plannerd"
    selfdrive/modeld/modeld.cc: vipc_client_main no frame
--- Errors on C2: Crash and black out
--- reboot C2 by hand
--- OK and NG: C2 doesn't have the folder "/data/media/0/realdata/" or "fcamera.hevc"
--- Read for "slow frame rate: 14.44 fps"
    /selfdrive/ui/mainJLL.cc
    main(int argc, char *argv[]) {
      MainWindow w;
        /selfdrive/ui/qt/window.cc
        MainWindow::MainWindow(QWidget *parent) : QWidget(parent) {
          homeWindow = new HomeWindow(this);
          /selfdrive/ui/qt/home.cc
          HomeWindow::HomeWindow(QWidget* parent) : QWidget(parent) {
            onroad = new OnroadWindow(this);
            /selfdrive/ui/qt/onroad.cc
            OnroadWindow::OnroadWindow(QWidget *parent) : QWidget(parent) {
              nvg = new AnnotatedCameraWidget(VISION_STREAM_ROAD, this);
                AnnotatedCameraWidget::paintGL() {  // Who calls this ???
                  cereal::ModelDataV2::Reader &model = sm["modelV2"].getModelV2();
                    // draw camera frame
                    // Wide or narrow cam dependent on speed
                  float v_ego = sm["carState"].getCarState().getVEgo();
                  if ((v_ego < 10) || s->wide_cam_only) {
                    wide_cam_requested = true;
                  CameraWidget::paintGL();
                  update_model(s, sm["modelV2"].getModelV2());
                  update_leads(s, radar_state, sm["modelV2"].getModelV2().getPosition());
                  drawLaneLines(painter, s);
                  drawLead(painter, lead_one, s->scene.lead_vertices[0]);
                  drawHud(painter);
                  LOGW("slow frame rate: %.2f fps", fps);
    /selfdrive/ui/mainJLL.cc
      /selfdrive/ui/qt/window.cc
        /selfdrive/ui/qt/home.cc
          /selfdrive/ui/qt/onroad.cc
--- Read for "fcamera.hevc"
    /selfdrive/loggerd/config.py
      ROOT = '/data/media/0/fakedata/'
      ROOT = '/data/media/0/realdata/'
    /selfdrive/loggerd/loggerd.h
      LogCameraInfo cameras_logged[] = {
        .filename = "fcamera.hevc",
    /selfdrive/loggerd/loggerd.cc
      struct LoggerdState {
      struct RemoteEncoder {
      std::unique_ptr<VideoWriter> writer;
      main(int argc, char** argv) {
        loggerd_thread();
          loggerd_thread() {
            msg_count = 0, bytes_count = 0;
            while (!do_exit) {
                while (!do_exit && (msg = sock->receive(true))) {
                  │if (qs.encoder) {
                    bytes_count += handle_encoder_msg(&s, msg, qs.name, remote_encoders[sock]);

                      handle_encoder_msg(LoggerdState *s, Message *msg, std::string &name, struct RemoteEncoder &re) {
                        LogCameraInfo &cam_info = (name == "driverEncodeData") ? cameras_logged[1] :
                          ((name == "qRoadEncodeData") ? qcam_info : cameras_logged[0]));
                        bytes_count = 0;
                          // extract the message
                        capnp::FlatArrayMessageReader cmsg(kj::ArrayPtr<capnp::word>((capnp::word *)msg->getData(), msg->getSize() / sizeof(capnp::word)));
                        auto event = cmsg.getRoot<cereal::Event>();
                        auto edata = (name == "driverEncodeData") ? event.getDriverEncodeData() :
                          ((name == "qRoadEncodeData") ? event.getQRoadEncodeData() : event.getRoadEncodeData()));
                          // encoderd can have started long before loggerd
                        re.writer.reset();
                        bytes_count += handle_encoder_msg(s, qmsg, name, re);
                        if (cam_info.record) {
                          re.writer.reset(new VideoWriter(s->segment_path,
                            cam_info.filename, idx.getType() != cereal::EncodeIndex::Type::FULL_H_E_V_C,
                            cam_info.frame_width, cam_info.frame_height, cam_info.fps, idx.getType()));
                          // write the header
                          auto header = edata.getHeader();
                          re.writer->write((uint8_t *)header.begin(), header.size(), idx.getTimestampEof()/1000, true, false);
                        if (re.writer) {
                          auto data = edata.getData();
                          re.writer->write((uint8_t *)data.begin(), data.size(), idx.getTimestampEof()/1000, false, flags & V4L2_BUF_FLAG_KEYFRAME);
                        MessageBuilder bmsg;
                        auto new_msg = bmsg.toBytes();
                        logger_log(&s->logger, (uint8_t *)new_msg.begin(), new_msg.size(), true);   // always in qlog?

                          /selfdrive/loggerd/logger.cc
                          logger_log(LoggerState *s, uint8_t* data, size_t data_size, bool in_qlog) {
                              lh_log(s->cur_handle, data, data_size, in_qlog);
                                lh_log(LoggerHandle* h, uint8_t* data, size_t data_size, bool in_qlog) {
                                  h->log->write(data, data_size);
                                    h->q_log->write(data, data_size);
                        bytes_count += new_msg.size();
                  │} else {
                    logger_log(&s.logger, (uint8_t *)msg->getData(), msg->getSize(), in_qlog);
                    bytes_count += msg->getSize();
    main(int argc, char** argv) {
      loggerd_thread() {
        handle_encoder_msg(LoggerdState *s, Message *msg, std::string &name, struct RemoteEncoder &re) {
          logger_log(LoggerState *s, uint8_t* data, size_t data_size, bool in_qlog) {
            lh_log(LoggerHandle* h, uint8_t* data, size_t data_size, bool in_qlog) {
              h->log->write(data, data_size);

230311: OK UI: PC: Change "cameraJLL.cc", scons Change Run "./replayJLL --data_dir"
--- Change "SConscript"
    /openpilot/tools/replay/SConscript
      replay_lib_src = ["replayJLL.cc", "routeJLL.cc", "consoleuiJLL.cc",
        "cameraJLL.cc", "framereaderJLL.cc", "filereader.cc", "logreader.cc", "util.cc"]
--- scons Run "./replayJLL --data_dir"
    (sconsvenv) jinn@Liu:~/openpilot$ scons
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
    (sconsvenv) jinn@Liu:~/openpilot/tools/replay$
      ./replayJLL --data_dir dataC "8bfda98c9c9e4291|2020-05-11--03-00-57--61"
      ./replayJLL --no-hw-decoder --data_dir dataC "8bfda98c9c9e4291|2020-05-11--03-00-57--61"
--- same Errors
--- Screen output
    Route: 8bfda98c9c9e4291|2020-05-11--03-00-57, 2 segments
    Car Fingerprint: TOYOTA PRIUS 2017
    │loading segment 61...                                                                                                                                           │
    │Failed to create specified HW device -1313558101.                                                                                                               │
    │No device with hardware decoder found. fallback to CPU decoding.                                                                                                │
    │paused... at 0 s                                                                                                                                                │
    │merge segments 61                                                                                                                                               │
    │camera[0] frame size 1164x874                                                                                                                                   │
    │Starting listener for: camerad
    //--- seconds =  0
--- Errors on Screen
    Error 1: │Error sending a packet for decoding: -1094995529                                                                                                                │
    Error 2: │camera[0] failed to get frame: 1                                                                                                                                │
--- Read "mainJLL.cc"
    /replay/mainJLL.cc
    main(int argc, char *argv[]) {
      │Replay *replay = new Replay(route, allow, block, nullptr, replay_flags, parser.value("data_dir"), &app);
        /replay/replayJLL.h
          #include "tools/replay/camera.h"
          #include "tools/replay/routeJLL.h"
          class Replay : public QObject {
        /replay/routeJLL.h
          struct RouteIdentifier {
            int segment_id;
          struct SegmentFile {
            QString road_cam;
          class Route {
            inline const RouteIdentifier &identifier() const { return route_; }
            bool loadFromLocal();
            RouteIdentifier route_ = {};
          class Segment : public QObject {
            Segment(int n, const SegmentFile &files, uint32_t flags, const std::set<cereal::Event::Which> &allow = {});
            const int seg_num = 0;
            std::unique_ptr<LogReader> log;
            std::unique_ptr<FrameReader> frames[MAX_CAMERAS] = {};
        /replay/replayJLL.cc
        Replay::Replay(QString route, QStringList allow, QStringList block, SubMaster *sm_, uint32_t flags, QString data_dir, QObject *parent)
      │replay->start(parser.value("start").toInt());
        Replay::start(int seconds) {
          seekTo(route_->identifier().segment_id * 60 + seconds, false);
            Replay::seekTo(double seconds, bool relative) {
              queueSegment();
                Replay::queueSegment() {
                  // load one segment at a time
                  │rDebug("loading segment %d...", n);
                  │seg = std::make_unique<Segment>(n, route_->at(n), flags_, allow_list);
                    /replay/routeJLL.cc
                    Segment::Segment(int n, const SegmentFile &files, uint32_t flags,
                      synchronizer_.addFuture(QtConcurrent::run(this, &Segment::loadFile, i, file_list[i].toStdString()));
                        Segment::loadFile(int id, const std::string file) {
                          frames[id] = std::make_unique<FrameReader>();
                          success = frames[id]->load(file, flags & REPLAY_FLAG_NO_HW_DECODER, &abort_, local_cache, 20 * 1024 * 1024, 3);
                            /replay/framereader.h
                            class FrameReader {
                              bool decode(int idx, uint8_t *yuv);
                              AVFrame * decodeFrame(AVPacket *pkt);
                              std::vector<AVPacket*> packets;
                                /tools/ubuntu_setup.sh
                                function install_ubuntu_common_requirements() { ffmpeg \
                                    // "FFmpeg libav tutorial - learn how media works":
                                    // The "AVPacket" is a slice of compressed data obtained from the "AVStream"
                                    //   that can be decoded by an "AVCodec".
                                    // https://github.com/FFmpeg/FFmpeg
                            /replay/framereader.cc
                            FrameReader::load(const std::byte *data, size_t size, bool no_hw_decoder, std::atomic<bool> *abort) {
                              AVStream *video = input_ctx->streams[0];
                              const AVCodec *decoder = avcodec_find_decoder(video->codecpar->codec_id);
                              decoder_ctx = avcodec_alloc_context3(decoder);
                              if (has_hw_decoder && !no_hw_decoder) {
                                  rWarning("No device with hardware decoder found. fallback to CPU decoding.");
                              packets.reserve(60 * 20);  // 20fps, one minute
                              AVPacket *pkt = av_packet_alloc();
                              packets.push_back(pkt);  // loading packets complete
                  │QObject::connect(seg.get(), &Segment::loadFinished, this, &Replay::segmentLoadFinished);
                  // start stream thread
                  │startStream(cur_segment.get());
                    Replay::startStream(const Segment *cur_segment) {
                      camera_server_ = std::make_unique<CameraServer>(camera_size);
                        /replay/cameraJLL.cc
                        CameraServer::CameraServer(std::pair<int, int> camera_size[MAX_CAMERAS]) {
                          startVipcServer();
                            CameraServer::startVipcServer() {
                              for (auto &cam : cameras_) {
                                rInfo("camera[%d] frame size %dx%d", cam.type, cam.width, cam.height);
                                cam.thread = std::thread(&CameraServer::cameraThread, this, std::ref(cam));
                                  CameraServer::cameraThread(Camera &cam) {                                                                                                                              │
                                    │auto read_frame = [&](FrameReader *fr, int frame_id) {
                                      VisionBuf *yuv_buf = vipc_server_->get_buffer(cam.stream_type);
                                      assert(yuv_buf);  // OK: yuv_buf
                                      bool ret = fr->get(frame_id, (uint8_t *)yuv_buf->addr);  // Error: ret = 0
                                      return ret ? yuv_buf : nullptr;  // Error: return nullptr > C++ 20  2023.3.15
                                        /replay/framereader.cc
                                        FrameReader::get(int idx, uint8_t *yuv) {
                                          assert(yuv != nullptr);  // OK: yuv != nullptr
                                          return decode(idx, yuv);
                                            FrameReader::decode(int idx, uint8_t *yuv) {
                                              AVFrame *f = decodeFrame(packets[i]);  // Error: packets[i]
                                                AVFrame *FrameReader::decodeFrame(AVPacket *pkt) {
                                                  int ret = avcodec_send_packet(decoder_ctx, pkt);  // Error: packets[i]
                                                  if (ret < 0) {
                                                    rError("Error sending a packet for decoding: %d", ret);
                                    │while (true) {
                                      const auto [fr, eidx] = cam.queue.pop();
                                      const int id = eidx.getSegmentId();
                                      bool prefetched = (id == cam.cached_id && eidx.getSegmentNum() == cam.cached_seg);
                                      auto yuv = prefetched ? cam.cached_buf : read_frame(fr, id);  //--- prefetched = 1  //--- cam.cached_buf = 0
                                      if (yuv) {  // Error: yuv = 0
                                      } else {
                                        rError("camera[%d] failed to get frame: %lu", cam.type, eidx.getSegmentId());
                                      cam.cached_id = id + 1;
                                      cam.cached_seg = eidx.getSegmentNum();
                                      cam.cached_buf = read_frame(fr, cam.cached_id);  // Error:
--- Read "mainJLL.cc" Summary
    /replay/mainJLL.cc
    main(int argc, char *argv[]) {
      /replay/replayJLL.cc
      Replay::start(int seconds) {
        Replay::seekTo(double seconds, bool relative) {
          Replay::queueSegment() {
            rDebug("loading segment %d...", n);
              /replay/routeJLL.cc
              Segment::Segment(int n, const SegmentFile &files, uint32_t flags,
                Segment::loadFile(int id, const std::string file) {
                  frames[id] = std::make_unique<FrameReader>();
                  success = frames[id]->load(file, flags & REPLAY_FLAG_NO_HW_DECODER, &abort_, local_cache, 20 * 1024 * 1024, 3);
                    /replay/framereader.cc
                    FrameReader::load(const std::byte *data, size_t size, bool no_hw_decoder, std::atomic<bool> *abort) {
                      AVStream *video = input_ctx->streams[0];
                      packets.reserve(60 * 20);  // 20fps, one minute
                      AVPacket *pkt = av_packet_alloc();
                      packets.push_back(pkt);  // loading complete
            │QObject::connect(seg.get(), &Segment::loadFinished, this, &Replay::segmentLoadFinished);
              Replay::startStream(const Segment *cur_segment) {
                  /replay/cameraJLL.cc
                  CameraServer::CameraServer(std::pair<int, int> camera_size[MAX_CAMERAS]) {
                      CameraServer::startVipcServer() {
                            CameraServer::cameraThread(Camera &cam) {                                                                                                                              │
                              auto read_frame = [&](FrameReader *fr, int frame_id) {
                              VisionBuf *yuv_buf = vipc_server_->get_buffer(cam.stream_type);
                              bool ret = fr->get(frame_id, (uint8_t *)yuv_buf->addr);  // Error: ret = 0
                              /replay/framereader.cc
                              FrameReader::get(int idx, uint8_t *yuv) {
                                  FrameReader::decode(int idx, uint8_t *yuv) {
                                        int ret = avcodec_send_packet(decoder_ctx, pkt);  // Error: packets[i]
                              while (true) {
                                bool prefetched = (id == cam.cached_id && eidx.getSegmentNum() == cam.cached_seg);
                                auto yuv = prefetched ? cam.cached_buf : read_frame(fr, id);  //--- prefetched = 1  //--- cam.cached_buf = 0
                                if (yuv) {  // Error: yuv = 0
                                } else {
                                  rError("camera[%d] failed to get frame: %lu", cam.type, eidx.getSegmentId());
                                cam.cached_id = id + 1;
                                cam.cached_seg = eidx.getSegmentNum();
                                cam.cached_buf = read_frame(fr, cam.cached_id);  // Error:
--- Pipe 1: /replay/mainJLL.cc > replayJLL.cc > routeJLL.cc > cameraJLL.cc >
            framereader.cc > /tools/ubuntu_setup.sh > ffmpeg > <libavcodec/avcodec.h>
--- Pipe 2: main() > Replay::start() > Replay::seekTo() > Replay::queueSegment() > Segment::Segment() >
    Segment::loadFile() > FrameReader::load() > decoder = avcodec_find_decoder() >
    decoder_ctx = avcodec_alloc_context3(decoder) > pkt = av_packet_alloc() >
    packets.push_back(pkt) > QObject::connect() > Replay::startStream() >
    CameraServer::CameraServer() > CameraServer::startVipcServer() >
    CameraServer::cameraThread() >
      FrameReader::get() > yuv_buf = vipc_server_->get_buffer() > FrameReader::get(*yuv) >
      FrameReader::decode() > FrameReader::decodeFrame() > avcodec_send_packet(decoder_ctx, pkt)
--- Read Screen output
    Route: 8bfda98c9c9e4291|2020-05-11--03-00-57, 2 segments
    Car Fingerprint: TOYOTA PRIUS 2017
      /replay/mainJLL.cc
      main(int argc, char *argv[]) {
        ConsoleUI console_ui(replay);
          /replay/consoleuiJLL.cc
          ConsoleUI::ConsoleUI(Replay *replay, QObject *parent) : replay(replay), sm({"carState", "liveParameters"}), QObject(parent) {
            initWindows();
              ConsoleUI::initWindows() {
                updateSummary();
                  ConsoleUI::updateSummary() {
                    mvwprintw(w[Win::Stats], 0, 0, "Route: %s, %lu segments", qPrintable(route->name()), route->segments().size());
                    mvwprintw(w[Win::Stats], 1, 0, "Car Fingerprint: %s", replay->carFingerprint().c_str());
--- Pipe 3: /replay/mainJLL.cc > consoleuiJLL.cc
--- Error 2: Debug output on screen
    //--- eidx.getSegmentId() = 0  // cereal::EncodeIndex::Reader& eidx
    //--- id = 0
    //--- cam.cached_id = -1
    //--- eidx.getSegmentNum() = 61
    //--- cam.cached_seg = -1
    //--- prefetched = 0
    //--- cam.cached_buf = 0
    //--- yuv = 0x1e5c110

    //--- eidx.getSegmentId() = 0
    //--- id = 0
    //--- cam.cached_id = 1
    //--- eidx.getSegmentNum() = 61
    //--- cam.cached_seg = 61
    //--- prefetched = 0
    //--- cam.cached_buf = 0x7f1c940372f0
    //--- yuv = 0x1e9fcd0

    //--- eidx.getSegmentId() = 1
    //--- id = 1
    //--- cam.cached_id = 1
    //--- eidx.getSegmentNum() = 61
    //--- cam.cached_seg = 61
    //--- prefetched = 1
    //--- cam.cached_buf = 0x1e9fd60
    //--- yuv = 0x1e9fd60

    //--- eidx.getSegmentId() = 6
    //--- id = 6
    //--- cam.cached_id = 6
    //--- eidx.getSegmentNum() = 61
    //--- cam.cached_seg = 61
    //--- prefetched = 1
    //--- cam.cached_buf = 0
      > error > read_frame(fr, cam.cached_id) > assert(yuv_buf)  // OK: yuv_buf
      > fr->get(frame_id, (uint8_t *)yuv_buf->addr)
      > AVFrame *f = decodeFrame(packets[i]);  // Error: packets[i]
      > rError("Error sending a packet for decoding: %d", ret)
        // C++ assert(int expression); If the argument expression equals zero
        //   (i.e., the expression is false), a message is written to the standard
        //   error device and abort is called, terminating the program execution.
    //--- yuv = 0
    camera[0] failed to get frame: 2

    //--- eidx.getSegmentId() = 0
    //--- cam.cached_id = -1
    //--- eidx.getSegmentNum() = 61
    //--- cam.cached_seg = -1
    //--- prefetched = 0
    //--- cam.cached_buf = 0
    //--- yuv = 0x1a34110
    //--- eidx.getSegmentId() = 0
    //--- cam.cached_id = 1
    //--- eidx.getSegmentNum() = 61
    //--- cam.cached_seg = 61
    //--- prefetched = 0
    //--- cam.cached_buf = 0x7f10d80372f0
    //--- yuv = 0x1a77cd0
    //--- eidx.getSegmentId() = 1
    //--- cam.cached_id = 1
    //--- eidx.getSegmentNum() = 61
    //--- cam.cached_seg = 61
    //--- prefetched = 1
    //--- cam.cached_buf = 0x1a77d60
    //--- yuv = 0x1a77d60
    //--- cam.cached_buf = 0x1b04870
    //--- yuv = 0x1b04870
    //--- cam.cached_buf = 0x1b04990
    //--- yuv = 0x1b04990
    //--- cam.cached_buf = 0x1b04990
    //--- yuv = 0x1b04990
    //--- cam.cached_buf = 0x1b04b40
    //--- yuv = 0x1b04b40
    //--- cam.cached_buf = 0x1b04b40
    //--- yuv = 0x1b04
    │Error sending a packet for decoding: -1094995529                                                                                                                │
    │avcodec_receive_frame error: -11
    STATUS:    loading...
--- Change "cameraJLL.cc"
    /openpilot/tools/replay/cameraJLL.cc
      if (yuv == 0 && cam.cached_buf != 0) { yuv = cam.cached_buf; }  // JLL
      if (yuv != 0 && cam.cached_buf == 0) { cam.cached_buf = yuv; }  // JLL
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
    (sconsvenv) jinn@Liu:~/openpilot/tools/replay$
      ./replayJLL --data_dir dataC "8bfda98c9c9e4291|2020-05-11--03-00-57--61"
--- OK: Error 2 and 230310 NG Solved!

230310: OK. NG. UI: PC: Change "routeJLL.cc", Run "./replayJLL --data_dir"
--- Change "routeJLL.cc"
    (sconsvenv) jinn@Liu:~/openpilot$ scons
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
    (sconsvenv) jinn@Liu:~/openpilot/tools/replay$
      ./replayJLL --data_dir dataC/8bfda98c9c9e4291/2020-05-11--03-00-57/61 "8bfda98c9c9e4291/2020-05-11--03-00-57/61"
      ./replayJLL --data_dir dataC "8bfda98c9c9e4291/2020-05-11--03-00-57/61"
      ./replayJLL --data_dir dataC/8bfda98c9c9e4291/2020-05-11--03-00-57/61 "8bfda98c9c9e4291/2020-05-11--03-00-57/61"
      ./replayJLL --data_dir dataC/8bfda98c9c9e4291/2020-05-11--03-00-57 "8bfda98c9c9e4291/2020-05-11--03-00-57--61"
      ./replayJLL --data_dir dataC "8bfda98c9c9e4291|2020-05-11--03-00-57--61"
      ./replayJLL --data_dir dataC "8bfda98c9c9e4291/2020-05-11--03-00-57--61"
      ./replayJLL --data_dir dataC/8bfda98c9c9e4291 "8bfda98c9c9e4291/2020-05-11--03-00-57--61"
      ./replayJLL --data_dir dataC/8bfda98c9c9e4291 "8bfda98c9c9e4291/2020-05-11--03-00-57/61"
        loading route  "8bfda98c9c9e4291/2020-05-11--03-00-57/61"
        //--- list[2] =  "/"
        //--- list[5] =  "61"
        //--- segment_id = list[5].toInt() =  61
        //--- route =  "8bfda98c9c9e4291/2020-05-11--03-00-57/61"
        //--- route_.str =  "8bfda98c9c9e4291|2020-05-11--03-00-57"
        //--- data_dir =  "dataC/8bfda98c9c9e4291"
        //--- data_dir_ =  "dataC/8bfda98c9c9e4291"
        //--- data_dir_.isEmpty() =  false
        //--- folder =  "2020-05-11--03-00-57"
        //--- route_.timestamp =  "2020-05-11--03-00-57"
        //--- seg_num =  61
        //--- segment_dir.entryList(QDir::Files) =  ()
        //--- !segments_.empty() =  false
        failed to load route "8bfda98c9c9e4291|2020-05-11--03-00-57" from "dataC/8bfda98c9c9e4291"
      ./replayJLL --data_dir "dataC/8bfda98c9c9e4291|2020-05-11--03-00-57" "8bfda98c9c9e4291|2020-05-11--03-00-57--61"
        //--- folder =  "61" == "62" (two folders have the same files (videos) for debugging)
          loading route  "8bfda98c9c9e4291|2020-05-11--03-00-57--61"
          //--- list[2] =  "|"
          //--- list[5] =  "61"
          //--- segment_id = list[5].toInt() =  61
          //--- route =  "8bfda98c9c9e4291|2020-05-11--03-00-57--61"
          //--- route_.str =  "8bfda98c9c9e4291|2020-05-11--03-00-57"
          //--- data_dir =  "dataC/8bfda98c9c9e4291|2020-05-11--03-00-57"
          //--- data_dir_ =  "dataC/8bfda98c9c9e4291|2020-05-11--03-00-57"
          //--- data_dir_.isEmpty() =  false
          //--- log_dir =  QDir( "dataC/8bfda98c9c9e4291|2020-05-11--03-00-57" , nameFilters = { "*" },  QDir::SortFlags( Name | IgnoreCase ) , QDir::Filters( Dirs|Files|Drives|AllEntries ) )
          //--- folder =  "61"
          //--- segment_dir =  QDir( "dataC/8bfda98c9c9e4291|2020-05-11--03-00-57/61" , nameFilters = { "*" },  QDir::SortFlags( Name | IgnoreCase ) , QDir::Filters( Dirs|Files|Drives|AllEntries ) )
          //--- segment_dir.entryList(QDir::Files) =  ("fcamera.hevc", "rlog.bz2")
          //--- segment_dir.absoluteFilePath(f) =  "/home/jinn/openpilot/tools/replay/dataC/8bfda98c9c9e4291|2020-05-11--03-00-57/61/fcamera.hevc"
          //--- segment_dir.absoluteFilePath(f) =  "/home/jinn/openpilot/tools/replay/dataC/8bfda98c9c9e4291|2020-05-11--03-00-57/61/rlog.bz2"
          //--- folder =  "62"
          //--- segment_dir =  QDir( "dataC/8bfda98c9c9e4291|2020-05-11--03-00-57/62" , nameFilters = { "*" },  QDir::SortFlags( Name | IgnoreCase ) , QDir::Filters( Dirs|Files|Drives|AllEntries ) )
          //--- segment_dir.entryList(QDir::Files) =  ("fcamera.hevc", "rlog.bz2")
          //--- segment_dir.absoluteFilePath(f) =  "/home/jinn/openpilot/tools/replay/dataC/8bfda98c9c9e4291|2020-05-11--03-00-57/62/fcamera.hevc"
          //--- segment_dir.absoluteFilePath(f) =  "/home/jinn/openpilot/tools/replay/dataC/8bfda98c9c9e4291|2020-05-11--03-00-57/62/rlog.bz2"
          //--- !segments_.empty() =  true
          load route 8bfda98c9c9e4291|2020-05-11--03-00-57 with 2 valid segments
--- OK. --data_dir
    ./replayJLL --data_dir dataC "8bfda98c9c9e4291|2020-05-11--03-00-57--61"
      //--- route =  "8bfda98c9c9e4291|2020-05-11--03-00-57--61"
      //--- data_dir 1 =  "dataC"
      //--- data_dir 2 =  "dataC/8bfda98c9c9e4291|2020-05-11--03-00-57"
      //--- route_.str =  "8bfda98c9c9e4291|2020-05-11--03-00-57"
      //--- data_dir_ =  "dataC/8bfda98c9c9e4291|2020-05-11--03-00-57"
      //--- log_dir =  QDir( "dataC/8bfda98c9c9e4291|2020-05-11--03-00-57" , nameFilters = { "*" },  QDir::SortFlags( Name | IgnoreCase ) , QDir::Filters( Dirs|Files|Drives|AllEntries ) )
      //--- folder =  "61"
      //--- segment_dir =  QDir( "dataC/8bfda98c9c9e4291|2020-05-11--03-00-57/61" , nameFilters = { "*" },  QDir::SortFlags( Name | IgnoreCase ) , QDir::Filters( Dirs|Files|Drives|AllEntries ) )
--- OK. Better. --data_dir
    ./replayJLL --data_dir dataC "8bfda98c9c9e4291|2020-05-11--03-00-57--61"
      loading route  "8bfda98c9c9e4291|2020-05-11--03-00-57--61"
      //--- route_.str =  "8bfda98c9c9e4291|2020-05-11--03-00-57"
      //--- data_dir_ =  "dataC/8bfda98c9c9e4291|2020-05-11--03-00-57"
      //--- folder =  "61"
      //--- folder =  "62"
      load route 8bfda98c9c9e4291|2020-05-11--03-00-57 with 2 valid segments
        Route: 8bfda98c9c9e4291|2020-05-11--03-00-57, 2 segments
        Car Fingerprint: TOYOTA "PRIUS" 2017
--- Solved 230307 NG ToDo
--- NG. UI shows but vedio is running then freezes.

230309: OK UI: PC: scons Change Run "./replayJLL --demo"
--- Copy "SConscript" to SConscript091 and Change (230307 ToDo)
    /tools/replay/SConscript
      replay_lib_src = ["replayJLL.cc", "routeJLL.cc", "consoleuiJLL.cc", ...
      qt_env.Program("replayJLL", ["mainJLL.cc"], ...
--- Change "SConstruct"
    /openpilot/SConstruct
      # 230309 build /tools/replay/mainJLL.cc
      SConscript(['tools/replay/SConscript'])
--- scons Run "replayJLL"
    (sconsvenv) jinn@Liu:~/openpilot$ scons
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
    (sconsvenv) jinn@Liu:~/openpilot/tools/replay$ ./replayJLL --demo
--- OK.
    loading route  "4cf7a6ad03080c90|2021-09-29--13-46-36"
    //--- route_.str =  "4cf7a6ad03080c90|2021-09-29--13-46-36"
    //--- data_dir_ =  ""
    load route 4cf7a6ad03080c90|2021-09-29--13-46-36 with 11 valid segments
      Route: 4cf7a6ad03080c90|2021-09-29--13-46-36, 11 segments
      Car Fingerprint: TOYOTA "RAV4" 2017

230307: OK. NG. ToDo. UI: PC: Run "./replay --demo, --data_dir"
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
  (sconsvenv) jinn@Liu:~/openpilot/tools/replay$
    ./replay --demo
      loading route  "4cf7a6ad03080c90|2021-09-29--13-46-36"
      load route 4cf7a6ad03080c90|2021-09-29--13-46-36 with 11 valid segments
--- OK. --demo
    ./replay --data_dir 8bfda98c9c9e4291|2020-05-11--03-00-57
      2020-05-11--03-00-57: command not found
    ./replay --data_dir dataA 8bfda98c9c9e4291
      loading route  "8bfda98c9c9e4291"
      invalid route format
    ./replay --data_dir dataA 8bfda98c9c9e4291|2020-05-11--03-00-57
      loading route  "8bfda98c9c9e4291"
      failed to load route "" from "dataA"
    ./replay --data_dir 8bfda98c9c9e4291 2020-05-11--03-00-57
      loading route  "2020-05-11--03-00-57"
      failed to load route "|2020-05-11--03-00-57" from "8bfda98c9c9e4291"
    ./replay --data_dir dataA 8bfda98c9c9e4291_2020-05-11--03-00-57
      loading route  "8bfda98c9c9e4291_2020-05-11--03-00-57"
      failed to load route "8bfda98c9c9e4291|2020-05-11--03-00-57" from "dataA"
    ./replay --data_dir dataA "8bfda98c9c9e4291|2020-05-11--03-00-57"
      loading route  "8bfda98c9c9e4291|2020-05-11--03-00-57"
      failed to load route "8bfda98c9c9e4291|2020-05-11--03-00-57" from "dataA"
    ./replay --data_dir dataA "8bfda98c9c9e4291|2020-05-11--03-00-57--61"
      loading route  "8bfda98c9c9e4291|2020-05-11--03-00-57--61"
      failed to load route "8bfda98c9c9e4291|2020-05-11--03-00-57" from "dataA"
    ./replay --data_dir "8bfda98c9c9e4291|2020-05-11--03-00-57" "--61"
      Unknown option '61'.
    ./replay --data_dir "8bfda98c9c9e4291|2020-05-11--03-00-57" "61"
      loading route  "61"
      invalid route format
      failed to load route "" from "8bfda98c9c9e4291|2020-05-11--03-00-57"
    ./replay --data_dir "8bfda98c9c9e4291|2020-05-11--03-00-57" --61
      Unknown option '61'.
    ./replay --data_dir dataA "8bfda98c9c9e4291|2020-05-11--03-00-57/61"
      loading route  "8bfda98c9c9e4291|2020-05-11--03-00-57/61"
      failed to load route "8bfda98c9c9e4291|2020-05-11--03-00-57" from "dataA"
    ./replay --data_dir /home/jinn/openpilot/tools/replay/dataA "8bfda98c9c9e4291|2020-05-11--03-00-57"
      loading route  "8bfda98c9c9e4291|2020-05-11--03-00-57"
      failed to load route "8bfda98c9c9e4291|2020-05-11--03-00-57" from "/home/jinn/openpilot/tools/replay/dataA"
    ./replay --data_dir dataA "8bfda98c9c9e4291|2020-05-11--03-00-57"
      loading route  "8bfda98c9c9e4291|2020-05-11--03-00-57"
      failed to load route "8bfda98c9c9e4291|2020-05-11--03-00-57" from "dataA"
--- NG. --data_dir
  /tools/replay/routeJLL.cc
    const QString DEMO_ROUTE = "4cf7a6ad03080c90|2021-09-29--13-46-36";
    QRegExp rx(R"(^(?:([a-z0-9]{16})([|_/]))?(\d{4}-\d{2}-\d{2}--\d{2}-\d{2}-\d{2})(?:(--|/)(\d*))?$)");
    return {.dongle_id = list[1], .timestamp = list[3], .segment_id = list[5].toInt(), .str = list[1] + "|" + list[3]};
      ^	The caret signifies the beginning of the string.
      $	The dollar signifies the end of the string.
      ?	Matches any single character. (?:pattern)
      [...]	Sets of characters can be represented in square brackets.
      .segment_id = list[5].toInt() is not assigned ???
        TEST_RLOG_URL = "https://commadataci.blob.core.windows.net/openpilotci/0c94aa1e1296d7c6/2021-05-05--19-48-37/0/rlog.bz2";
        http.sendRequest("https://api.commadotai.com/v1/route/" + route_.str + "/files");
        comma_cache = util::getenv("COMMA_CACHE", "/tmp/comma_download_cache/");
--- clean up "comma_download_cache"
  root@Liu:/tmp/comma_download_cache# ls -sh
    total 419M
  root@Liu:/tmp# rm -rf comma_download_cache
--- Read code for ./replay --data_dir dataA "8bfda98c9c9e4291|2020-05-11--03-00-57"
  /replay/main.cc >
  main(int argc, char *argv[]) {
    parser.process(app);
    const QStringList args = parser.positionalArguments();
    const QString route = args.empty() ? DEMO_ROUTE : args.first();
      > route = "8bfda98c9c9e4291|2020-05-11--03-00-57"
    Replay *replay = new Replay(route, , , , , parser.value("data_dir"), );
      > parser.value("data_dir") = "dataA"
      Replay::Replay(QString route, , , , QString data_dir, ) : sm, , {
        > route = "8bfd...", data_dir = "dataA"
        std::unique_ptr<Route> route_;
        route_ = std::make_unique<Route>(route, data_dir);
          /replay/route.cc >
          Route::Route(const QString &route, const QString &data_dir) : data_dir_(data_dir) {
            QString data_dir_; > data_dir_ = "dataA", route = "8bfd..."
            RouteIdentifier route_ = {};
            route_ = parseRoute(route);
              Route::parseRoute(const QString &str) {
                > str = "8bfda98c9c9e4291|2020-05-11--03-00-57"
                QRegExp rx(R"(^(?:([a-z0-9]{16})([|_/]))?(\d{4}-\d{2}-\d{2}--\d{2}-\d{2}-\d{2})(?:(--|/)(\d*))?$)");
                const QStringList list = rx.capturedTexts();
                return {.dongle_id = list[1], .timestamp = list[3], .segment_id = list[5].toInt(), .str = list[1] + "|" + list[3]};
            > route_ = {.dongle_id = "8bfda98c9c9e4291", .timestamp = "2020-05-11--03-00-57",
                        .segment_id = 030057???, .str = "8bfda98c9c9e4291|2020-05-11--03-00-57"}
        > route_ = {.dongle_id = "8bfda98c9c9e4291", .timestamp = "2020-05-11--03-00-57",
                    .segment_id = 030057???, .str = "8bfda98c9c9e4291|2020-05-11--03-00-57"}
    if (!replay->load()) {
      Replay::load() {
        route_->load() >
          Route::load() {
            loadFromLocal();
              std::map<int, SegmentFile> segments_;
              Route::loadFromLocal() {
                return !segments_.empty();
                   > segments_.empty() = True >
        route_->load() = False
        route_->name() >
          QString &name() const { return route_.str; }
        route_->name() = "8bfda98c9c9e4291|2020-05-11--03-00-57"
        route_->dir() > QString &dir() const { return data_dir_; } >
        route_->dir() = data_dir_ = "dataA"
        if (!route_->load()) { qCritical() << "failed to load route" << route_->name()
                                           << "from" << (route_->dir().isEmpty() ? "server" : route_->dir());
                               return false;
--- Read code Summary
  /replay/main.cc >
  main(int argc, char *argv[]) {
      > route = "8bfda98c9c9e4291|2020-05-11--03-00-57"
    Replay *replay = new Replay(route, , , , , parser.value("data_dir"), );
      > parser.value("data_dir") = "dataA"
      Replay::Replay(QString route, , , , QString data_dir, ) : sm, , {
        > route = "8bfd...", data_dir = "dataA"
          /replay/route.cc >
          Route::Route(const QString &route, const QString &data_dir) : data_dir_(data_dir) {
              Route::parseRoute(const QString &str) {
                > str = "8bfda98c9c9e4291|2020-05-11--03-00-57"
                QRegExp rx(R"(^(?:([a-z0-9]{16})([|_/]))?(\d{4}-\d{2}-\d{2}--\d{2}-\d{2}-\d{2})(?:(--|/)(\d*))?$)");
                return {.dongle_id = list[1], .timestamp = list[3], .segment_id = list[5].toInt(), .str = list[1] + "|" + list[3]};
        > route_ = {.dongle_id = "8bfda98c9c9e4291", .timestamp = "2020-05-11--03-00-57",
    if (!replay->load()) {
      Replay::load() {
          Route::load() {
              Route::loadFromLocal() {
                return !segments_.empty();
        route_->name() = "8bfda98c9c9e4291|2020-05-11--03-00-57"
        route_->dir() = data_dir_ = "dataA"
--- Pipe: /replay/main.cc > /replay/route.cc
--- ToDo: Change /replay/route.cc Route::loadFromLocal

230306: OK UI: PC: Change Run "cycle_alertsJLL.py", "Joystick Mode", "testJoystick"
--- Change to cycle_alertsJLL.py
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
    from selfdrive.controls.lib.events import Events, ET
    current_alert_types = [ET.PERMANENT]
    current_alert_types.append(ET.WARNING)
    current_alert = True
    if current_alert:
      controlsState.alertText1 = current_alert.alert_text_1
      controlsState.alertText2 = current_alert.alert_text_2
      controlsState.alertSize = current_alert.alert_size
      controlsState.alertStatus = current_alert.alert_status
      controlsState.alertBlinkingRate = current_alert.alert_rate
      controlsState.alertType = current_alert.alert_type
      controlsState.alertopenpilot/aJLL/UI$ python cycle_alertsJLL.py

  (sconsvenv) jinn@Liu:~/openpilot$ python cycle_alertsJLL.py
--- OK. Solved UI 1: "Joystick Mode" on PC

230305: OK. NG. UI: PC: Run "joystickdJLL.py  --keyboard"
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
    #Params().put_bool('JoystickDebugMode', True)  # controlsd.py uses it not UI

    EventName = car.CarEvent.EventName
    events = Events()
    events.add(EventName.joystickDebug, static=True)
    car_events = events.to_msg()
    msg = messaging.new_message('carState')
    msg.valid = True
    msg.carState = car.CarState.new_message()
    msg.carState.events = car_events
      #print('#--- car_events =', car_events)
      #--- car_events = [<car.capnp:CarEvent builder (name = joystickDebug,
      #  enable = false, noEntry = false, warning = true, userDisable = false,
      #  softDisable = false, immediateDisable = false, preEnable = false,
      #  permanent = true, overrideLongitudinal = false, overrideLateral = false)>]
    pm.send('carState', msg)
--- NG. UI doesn't sm 'carState' for "Joystick Mode"
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python bodyJLL.py
py
--- NG. UI doesn't use joystickd.py for "Joystick Mode"
  (sconsvenv) jinn@Liu:~/openpilot$ python bodyJLL.py
--- OK.
  (sconsvenv) jinn@Liu:~/openpilot$ python joystickdJLL.py  --keyboard
--- NG:
  Params().put_bool('JoystickDebugMode', True)  # controlsd.py uses it not UI
  UI doesn't sm 'carState', 'carEvents', 'carParams', 'controlsState' for "Joystick Mode"
  UI doesn't use joystickd.py for "Joystick Mode"

========== Appendix ==========

----- Tutorials

--- W3Schools Online Web Tutorials (Python, C++, ...)
--- Shell Scripting for Beginners – How to Write Bash Scripts in Linux
--- SCons User Guide 1.2.0: Chapter 2. Simple Builds
--- Capnp.Tutorial - Hackage - Haskell.org
--- ZeroMQ | Get started
--- Qt5 Tutorial Hello World - 2020
--- OpenCL: A Hands-on Introduction

"callback function" in Python is a function that is an input argument passed to another function.
