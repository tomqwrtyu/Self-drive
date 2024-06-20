CB 8  Jinn-Liang Liu  2024.2.14 -      All Rights Reserved. © Copyright 2024

========== OP Coding Notes: Run, Change, Read ==========

"openpilot (OP)", "dragonpilot (DP)", "legacypilot (LP)",
"Comma Two (C2A/C2B)", and "Comma Body (CB)" Notes

OP/DP/LP Versions:
  DP091: C2 Version 0.9.1 (2022-12-XX) downloaded on 2023-01-17
  OP091: PC Version 0.9.1 (2022-12-XX) downloaded on 2023-01-19
  LP092: C2/PC "legacypilot" by Rick Lan          on 230426

Abbreviations: Ctr, Md, UI
  AR (Autonomous Rolling), CAN (Control Area Network), CBQ (CB Question),
  Ctr (Control), Lcz (Localization), Md (Model), Msg (Message), Net (Network), Nav (Navigation),
  UI (User Interface), XJ (Changed by JLL),
  PID (Proportional-Integral-Derivative Control in [Liu23a]),
  MPC (Model Predictive Control in [Liu23b]),
  KF  (Kalman Filter Control in [Liu23c])
  > (implies or next step), >! (<!) (better (worse) than), MC1 (Md Case 1), S 1 (Summary 1)

[Liu23a] J.-L. Liu, "AI-Based PID Control for Autonomous Driving", 2023
[Liu23b] J.-L. Liu, "AI-Based Model Predictive Control for Autonomous Driving", 2023
[Liu23c] J.-L. Liu, "AI-Based Kalman Filter Control for Autonomous Driving", 2023

===== ToDo Problems =====
  RT 1:  220208 Road Test Error on modelB5C2C.dlc:                 Solved 240121
  RT 2:  220426 Road Test Error: "No close lead car"               Solved 240121
  RT 3:  240121 Road Test Error: "Path Prediction Error"
  RT 4:  240204 Road Test Error on SimCM5f1.dlc: same as RT 2
  Ctr 1: Slow CB "Speed" ("Control") down                          Solved 230417
  Ctr 2: "Joystick Mode" > "Green Ball" > CB moves                 Solved 230509
  Ctr 3: CB rolled grossly from (0,0) to (3.24, -0.57)m
  Md 1:  'modelV2' + 'longitudinalPlan' + 'lateralPlan'
  Md 2:  "supercombo079.summary" > write "modelB7.py"
  Md 3:  aJLL/tinygrad/test/test_nn.py
  Md 4a: "h5toonnx.py" > "modelB6.onnx"                            Solved 231113
  Md 4b: "onnx2py.py" > "supercombo092.py"                         Solved 231211
  Md 4d: (OK Md 4a) + (OK Md 4b) + (NG Md 4c) > Hand code py
  UI 0:  "openpilot Unavailable"                                   Solved 230412
  UI 1:  "Joystick Mode" replaces "Body Face", "Battery fuelGauge" Solved 230505
  UI 2a: How to display PC/C2 UI run by modelB6.onnx ???
  UI 2b: 240116: 樹德科技大學 Saber: "Simulation"
  UI 2c: 231203: 第四屆台灣OP年會 Wang: "Simulation 1, 2"
  UI 2d: Add "啟用行車記錄功能" to OP079C2 for "B6CM2.dlc"
  UI 3:  CBQ 6 + CBQ 7 + CBQ 8

  Msg 1: "cereal/messaging" + "capnp" > create "/cerealJLL"
  AR 2:  CB rolls a) forward/backward 3 m, b) fw/turn/bw/tr 180
  AR 3:  CB moves from Point A to B with PID 1 in (X, Y) system
  Nav 1: /selfdrive/assets/navigation/screenshot.png, GPSNMEAData
  AM 1:  AR 2 + Md 1 (Path)
  AM 2:  AR 3 + Md 1 (PID 1 + Path)
  AMN 1: AR 3 + Md 1 + Nav 1 (PID 1 + Path + GPS)
  Lcz 1: Change "self.path_xyz = " to "straight w/o md" for CBQ 2
  PID 1: [Liu23a] + Apdx 1-4 CB5.py
  PID 2: [Liu23a] + "PIDController" + "LatControlPID" + "LatControlTorque"
  MPC 1: [Liu23b] + "mpc.py" A. Sakai
  MPC 2: [Liu23b] + 'lat_mpc.py'
  KF 1:  [Liu23c] + kalman1.py
  KF 2:  [Liu23c] + 'kalman2a.py' + 'kalman2.py' + 'kalman3.py' + 'radardJLL.py'
  CBQ 1: Why is CB so speedy when ignized, Why vibrates a lot,
         How to calibrate it ???                                   Solved 230502
  CBQ 2: = Ctr 3 How to keep CB rolling straight w/o turning ???
  CBQ 3: Somtimes body On => body Off => C2 black screen ???
  CBQ 5: "Fan Malfunction", "fanMalfunction"                       Solved 230509
  CBQ 6: How to draw "self.path_xyz" on PC ???
  CBQ 7: Why Output 1 != Output 2 (Error in def draw_path) ???
  CBQ 8: How to draw CB's "self.path_xyz" on C2 ???
  CBQ 9: CB keeps going forward when self.odometer > 6.5 Why ???   Solved 230912
  Data:  /data/media/0/realdata still logging

. ~/sconsvenv/bin/activate
print('#--- vars(kalman_params) =', vars(kalman_params))  # Python: Print an Object’s Attributes
print('#--- len(self.params) =', len(self.params))
print("#--- cyaw2 =", [ round(elem, 2) for elem in cyaw2 ])
print("%2.2f sec   %6.2f m  %6.2f m/s  %6.2f m/s2   lead_rel: %6.2f m  %6.2f m/s"
      % (self.current_time, self.distance, self.speed, self.acceleration, d_rel, v_rel))

--- disk free space
    root@localhost:/data$ df -ha
      24G  3.5G   20G  15% /data
      /data/media/0/realdata
--- size of folders and files
    root@localhost:/data$ du -hsc *
--- recursively remove files and folders
    root@localhost:/data$ rm -rf openpilotXYZ
--- recursively remove all files
    root@localhost:/data/openpilot/installer/media/0/dashcam$ rm -r *
--- remove "__pycache__" folders or "*.o" files
    jinn@Liu:~/DP2304PC$ find . -type d -name  "__pycache__" -exec rm -r {} +
    $ find . -name '*.a' -delete  # '*.o', '*.os', '*.pyc', 'moc_*', '__pycache__'
--- check cpu
    jinn@Liu:~$ cat /proc/cpuinfo
--- check gpu
    jinn@Liu:~$ sudo lshw -C display
--- copy .pyenv_OP091 to .pyenv
    jinn@Liu:~$ cp -avr .pyenv_OP091 .pyenv
--- clone OP092
    jinn@Liu:~$ git clone -b v0.9.2 https://github.com/commaai/openpilot
--- clean up "comma_download_cache"

========== Contents of Important Coding Notes ==========
190828: ===== DH Lee's Report on Comma2k19 =====
200428: ===== Toyota Prius Fingerprint OK =====
220208: OK NG C2A+car: Road Test on "modelB5JL220208.dlc"
230110: ===== Comma Body Begin =====
230118: OK Install: DP091 = DPbeta2 on C2+body/car
230213: Ctr 1: Pipe 1-3: PC: Read "controlsd.py", "Speed"
230222: OK UI: PC: scons Change Run "_uiJLL", "./replay --demo"
230225: OK Wang: SnG: PC+C2+body: "joystickd.py", "Joystick Mode", "testJoystick"
230303: OK UI: PC+C2: Run "uiview.py"
230304: OK UI: PC: Run "uiviewJLL.py", "bodyJLL.py"
230306: OK UI: PC: Change Run "cycle_alertsJLL.py", "Joystick Mode", "testJoystick"
230309: OK UI: PC: scons Change Run "./replayJLL --demo"
230311: OK UI: PC: Change "cameraJLL.cc", scons Change Run "./replayJLL --data_dir"
230317: OK NG UI: PC+C2: Run "uiview.py" ("Green Path" on C2. No "fcamera.hevc")
230318: OK Run Car+C2: Use "Enable On-Road Dashcam", "啟用行車記錄功能"
230329: OK scons PC: "DP091aL" using ".pyenv_DP091L", "DP091venv"
230404: OK scons PC: "DP091L = DP091aL" using "/.pyenv_OP091", "/sconsvenv_OP091"
230411: ===== Talk for Saber's Blog =====
230412: OK UI 0: PC: DP091L: "uiviewJLL.py", "bodyJLL.py", 'controlsState'
230416: OK. Got Assembled my "body"
230417: OK Ctr 1: CBQ 1: PC+C2+body: "controlsd.py", "Speed"
230419: OK CBQ 1: XJ 1-2: PC+C2+body: "controlsdXJ.py", XJ: Changed by JLL
230427: OK scons PC+C2: Install "LP092" (legacypilot) by Rick Lan
230502: OK UI 1 CBQ 1: Data: XJ 1-3: CBQ 2-4: PC+C2+body: "realdata", Change "LP092XJ"
230505: OK UI 1: XJ 4: CBQ 5: C2+body: Change "onroad.cc"
230509: OK CBQ 5, Ctr 2: XJ 5: C2+body: Change "controlsd.py"
230521: AR 2: C2+body: Read "self.LoC", "self.LaC", "PIDController"
230525: OK AR 2a: XJ 6: Ctr 3: Nav 1: AMN 1: C2+body: Change "controlsd.py", "carcontroller.py"
230529: NG Ctr 1a: PID 1: AR 3: C2+body: Change "controlsd.py", "carcontroller.py"
230530: OK MPC 1: PC: Run Read "mpc.py"
230619: OK MPC 2: Pipe 1f: PC: Read 'lat_mpc.py','lateral_planner.py' Run 'test_lateral_mpcJLL.py
230702: OK Run PC: Pipe 1b 3b: Run Read 'run_paramsd_on_routeJLL.py' for 'paramsd.py'
230708: OK KF 1-2: Pipe 1 3: PC: Run 'kalman1.py', 'kalman2.py', 'kalman3.py'
230714: OK Msg 1: Pipe 1a 1b: PC: scons "/cerealJLL", Run "controlsdJLL.py", "controlsdJLL1.py"
230719: OK AR 2: Md 1: XJ 7: Pipe 1: PC+C2+body: Change "controlsd.py", Rewrite "longcontrol.py", "latcontrol.py"
230725: OK AM 1: NG CBQ 2: XJ 8: Lcz 1: Pipe 1-6: PC+C2+body: Change "controlsd.py", "carcontroller.py"
230814: OK PC: Run "plantLonJLL.py", "plantLatJLL.py"
230822: OK UI: CBQ 6,7: PC: scons Run "./replayJLL --demo --data_dir" + "uiJLL.py", "uiviewJLL.py", or "/ui/ui"
230825: OK PC: Add "PYTHONPATH" to ".bashrc" for running "aJLL/ModelOld/dbcJLL.py" (not /openpilot)
230826: NG RT 2: PC: Run "modelB6", "ModelOld"
230912: OK CBQ 9: AR 2: XJ 9: PC+C2+body: Change "controlsd.py" "longcontrol.py" "carcontroller.py"
230913: OK. Md 3: PC: Change Run "tiny1.py" "tiny2.py"
230921: ===== Meet George Hotz =====
230922: OK "VMware" PC: openpilot$ "scons -i" then "scons -u -j$(nproc)"
231107: NG RT 2: C2A+car: Road Test: "supercombo079.dlc", "B5YJ0421.dlc"
231113: OK Md 4a: NG UI 2a: PC: "h5toonnx.py" > "modelB6.onnx"
231121: PID 2: Pipe 1-3: PC: Read "LatControlTorque" + Run Read "test_latcontrolJLL.py"
231203: ===== 第四屆台灣OP年會 =====
231211: OK Md 4b: NG Md 4c: Md 4d: PC: "onnx2py.py" > "supercombo092.py"
231217: OK UI: PC: Run Read "PlotJuggler"
231229: OK Pipe 1: PC: Run Change "train_modelB6.py", "custom_loss"
240109: Poster Presentation: 2309 AI Projects.docx
240113: PC: Run OP095/"get_model_metadata.py"
240116: ===== 樹德科技大學 =====
240121: OK RT 2: RT 3: UI 2d: C2A+car: Road Test: "B6CM2.dlc"
240124: NG MC3 MC4 MC5: NG RT 4: Pipe A B: S 1 S 2 S 3: PC: Run Change "train_modelB6.py"
240207: NG MC3 MC6 MC7: PC: Run Change "train_modelB6.py"

========== Summary of Coding Notes ==========

===== ToDo =====
24xx: Lcz 1 + AM 1: XJ 10: PC+C2+body: Change "controlsd.py"
--- Do AM 1 + CBQ 2 + Lcz 1: 230619 + 230725 + 230814 + 230822
    Reset "self.path_xyz =, modelV2.position, .orientation, .orientationRate" as in plantLatJLL.py
24xx: CBQ 6: PC: Change ""
--- Do CBQ 6: How to draw "self.path_xyz" on PC ???
--- Read "draw_path"
    tools/replay/lib/ui_helpers.py
    def draw_path(path, color, img, calibration, top_down, lid_color=None, z_off=0):
===== ToDo =====  <  <  <  <  <  >  >  >  >  >  >  >  >

===== Details of Coding Notes =====

240218: UI 2b: PC+C2: "interface.py",
--- Do UI 2b: 240116: 樹德科技大學 Saber: "Simulation"
--- Setup: C2 + harness + adaptor (Model: ZXD-1220, 12V/2A) + Type C cable
--- OK: C2 Screen: “openpilot Unavailable Waiting for controls to start”
--- Change Line 314 /home/jinn/CarXJ/SimXJ/selfdrive/car/toyota/interface.py
    #ret.canValid = self.cp.can_valid and self.cp_cam.can_valid
--- Change Lines 256-274 /home/jinn/CarXJ/SimXJ/selfdrive/ui/ui.cc
    /*if (s->started && !s->scene.frontview && ((s->sm)->frame - s->started_frame) > 5*UI_FREQ) {
    }*/
--- NG OK: C2 Screen shows lane lines but no path line
--- Change Line 47 /home/jinn/CarXJ/SimXJ/selfdrive/car/toyota/carstate.py
    ret.vEgo = 5.0
--- NG: no path line > undo
--- Change Line 46 /home/jinn/CarXJ/SimXJ/selfdrive/car/toyota/carstate.py
    ret.vEgoRaw = 5.0
--- NG: no path line > undo
--- Read scene.model.getPath()
    /home/jinn/OP/OP079C2/SConstruct
    if qt_env is None:
    if arch in ["x86_64", "Darwin", "larch64"]:
      qt_env = env.Clone()
      /home/jinn/OP/OP079C2/selfdrive/ui/SConscript
--- android = C2:
      if qt_env is None:
        src += ["android/ui.cc", "android/sl_sound.cc"]
        env.Program('_ui', src,
          selfdrive/ui/android/ui.cc
          int main(int argc, char* argv[]) {
            UIState uistate = {};
              typedef struct UIScene {
                float path_points[MODEL_PATH_DISTANCE];
            UIState *s = &uistate;
            printf("pvd:%d",s->model_path_vertices[0].cnt);
            ui_init(s);
            while (!do_exit) {
              ui_update(s);
              ui_draw(s);
                selfdrive/ui/ui.cc
                void ui_update(UIState *s) {
                  update_sockets(s);
                    void update_sockets(UIState *s) {
                      UIScene &scene = s->scene;
                        fill_path_points(scene.model.getLeftLane(), scene.left_lane_points);
                        fill_path_points(scene.model.getPath(), scene.path_points);
                          static inline void fill_path_points(const cereal::ModelData::PathData::Reader &path, float *points) {
                            const capnp::List<float>::Reader &poly = path.getPoly();
                            for (int i = 0; i < path.getValidLen(); i++) {
                              points[i] = poly[0] * (i * i * i) + poly[1] * (i * i) + poly[2] * i + poly[3];
                selfdrive/ui/paint.cc
                void ui_draw(UIState *s) {
                  ui_draw_vision(s);
                    static void ui_draw_vision(UIState *s) {
--- PC:
      else:
        qt_src = ["qt/ui.cc", "qt/window.cc", "qt/settings.cc", "qt/qt_sound.cc"] + src
        qt_env.Program("_ui", qt_src, LIBS=qt_libs + libs)
          selfdrive/ui/main.cc
          int main(int argc, char *argv[]) {
            MainWindow w;
            setMainWindow(&w);
              selfdrive/ui/qt/window.cc
              MainWindow::MainWindow(QWidget *parent) : QWidget(parent) {
                GLWindow * glWindow = new GLWindow(this);
              GLWindow::GLWindow(QWidget *parent) : QOpenGLWidget(parent) {
                QObject::connect(timer, SIGNAL(timeout()), this, SLOT(timerUpdate()));
                  void GLWindow::initializeGL() {
                    ui_state = new UIState();
                  void GLWindow::timerUpdate(){
                    ui_update(ui_state);
--- Pipe: ui_update(s) > scene.model.getPath()

========== Appendix (Apdx) ==========
