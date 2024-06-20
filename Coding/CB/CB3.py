CB 3  Jinn-Liang Liu  2023.3.24 - 4.18  All Rights Reserved. © Copyright 2023

========== OP Coding Notes: Run, Change, Read ==========

"openpilot (OP)", "dragonpilot (DP)", "Comma Two (C2A/C2B)", and "Comma Body (CB)" Notes

OP/DP Versions:
  1. DP091: C2    Version 0.9.1 (2022-12-XX) downloaded on 2023-01-17
  2. OP091: PC    Version 0.9.1 (2022-12-XX) downloaded on 2023-01-19
  3. DP091a: PC   Version 0.9.1 (2023-02-23) downloaded on 2023-03-27
  4. DP091aL: PC  Long DP091a changed by JLL            on 230327

Abbreviations: CAN, Ctr, Md, Nav, UI
  AR (Autonomous Rolling), CAN (Control Area Network), CBQ (CB Question), Ctr (Control),
  Md (Model), Nav (Navigation), UI (User Interface), XJ (Changed by JLL),

. ~/sconsvenv/bin/activate

========== Summary of Coding Notes ==========

230417: OK Ctr 1: CBQ 1: PC+C2+body: "controlsd.py", "Speed"
  root@localhost:/data/openpilot/tools/joystick$ python web.py
--- http://10.0.0.4:5000/
--- Change Line 638 /controls/controlsd.py:
          actuators.accel = 1.5*clip(self.sm['testJoystick'].axes[0], -1, 1)  # JLL OK
--- OK. Solved Ctr 1 "Speed"
--- Prob 4: body vibrates a lot in calibration due to "PIDController" ???
--- Prob 5: body cannot go straight line ???

230417a: NG. Ctr: iPhone+PC+C2+body: "QR code"
--- A. iPhone: scan QR code on C2
--- this is for Mapbox navigation NOT for ssh to C2
--- B. iPhone: Termius ssh
--- ssh: Edit c2key089.ppk > Passphrase: sxxxxxx6 > tap Save
  > Hosts > C2 > passphrase: sxxxxxx6 > tap Connect > Authentication failed (publickey) >
--- NG: Termius ssh

230416: OK. Got Assembled my "body"

230413: OK UI: PC: DP091L: redo 230306
--- rename /openpilot/selfdrive/car to ../car_DP091
--- move /OP091/selfdrive/car to ../car
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python cycle_alertsJLL.py

--- save /openpilot/aJLL/UI/uiviewJLL.py to ../uiviewJLL1.py

230412: OK UI 0: PC: DP091L: "uiviewJLL.py", "bodyJLL.py", 'controlsState'
--- Change uiviewJLL.py
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python bodyJLL.py
  (sconsvenv) jinn@Liu:~/openpilot$ python joystickdJLL.py  --keyboard

230412a: OK UI: PC: DP091L: ['ui'], Solved 230410b ???
--- ['ui'] in uiviewJLL.py
--- rename /openpilot/selfdrive/ui/ui to ../ui_DP091
--- Copy /OP091/selfdrive/ui/ui to /openpilot/../ui > Solved ???
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
  (sconsvenv) jinn@Liu:~/openpilot/tools/replay$
    ./replayJLL --data_dir dataC "8bfda98c9c9e4291|2020-05-11--03-00-57--61"

230411: ===== Talk for Saber's Blog =====

230410b: OK. ??? Redo 230311 on DP091L = DP091aL
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
  (sconsvenv) jinn@Liu:~/openpilot/tools/replay$
    ./replayJLL --data_dir dataC "8bfda98c9c9e4291|2020-05-11--03-00-57--61"

230410a: OK Install: "Kazam"

230406: OK. ??? UI: PC: DP091L: Change Run "launch_openpilotJLL.sh" from 230330
  (sconsvenv) jinn@Liu:~/openpilot$ bash launch_openpilotJLL.sh
    cd selfdrive/manager && ./manager.py
--- Pipe Cython: params.h > params.cc > params.o > params_pyx.pyx > params_pyx.so >
    params_pyx.cpp > params_pyx.o > params.py
--- Change common/api/__init__.py Line 10 from OP091
--- Change /selfdrive/controls/lib/events.py to events_DP091.py
--- Copy /OP091/../events.py to /openpilot/                ../events.py
--- Copy /OP091/selfdrive/manager/manager.py to            ../manager.py
--- Change /selfdrive/manager/manager.py Line 32
--- Copy
  /OP091/selfdrive/manager/process.py to                   ../process.py
  /OP091/selfdrive/manager/process_config.py to            ../process_config.py
  /OP091/selfdrive/controls/lib/longitudinal_planner.py to ../longitudinal_planner.py
  /OP091/selfdrive/controls/lib/lateral_planner.py to      ../lateral_planner.py
  /OP091/selfdrive/controls/plannerd.py to                 ../plannerd.py
  /OP091/selfdrive/controls/radard.py to                   ../radard.py
--- Pipe: launch_openpilotJLL.sh > ./manager.py > logmessaged, ui, soundd, deleter,
    thermald, statsd (no "loggerd" as 230319 ???)

230404: OK scons PC: "DP091L = DP091aL" using "/.pyenv_OP091", "/sconsvenv_OP091"
--- Change /openpilot to /OP091 and /DP091L to /openpilot
  jinn@Liu:~$ cp -avr .pyenv_OP091 .pyenv
  jinn@Liu:~$ cp -avr sconsvenv sconsvenv_OP091
--- Backups: .pyenv_OP091, sconsvenv_OP091
  (sconsvenv) jinn@Liu:~/openpilot$ tools/ubuntu_setup.sh
  (sconsvenv) jinn@Liu:~/openpilot$ scons -u -j$(nproc)
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
  (sconsvenv) jinn@Liu:~/openpilot/tools/replay$
    ./replayJLL --data_dir dataC "8bfda98c9c9e4291|2020-05-11--03-00-57--61"

230404a: OK UI: PC: OP091: Redo 230324 Ch4.
--- Change /openpilot to /DP091L = /DP091aL and /OP091 to /openpilot
--- rename .pyenv to .pyenv_DP091L and .pyenv_OP091 to .pyenv
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
  (sconsvenv) jinn@Liu:~/openpilot/tools/replay$
    ./replayJLL --data_dir dataC "8bfda98c9c9e4291|2020-05-11--03-00-57--61"

230330: NG. UI: PC: DP091aL: Do 230329a ??? by "launch_openpilotJLL.sh"
  (DP091venv) jinn@Liu:~/openpilot/selfdrive/manager$ ./manager.py
--- Error
--- use "launch_openpilot.sh"
  (DP091venv) jinn@Liu:~/openpilot$ bash launch_openpilotJLL.sh
  /openpilot/tools/sim/launch_openpilot.sh
    DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
    cd ../../selfdrive/manager && exec ./manager.py
xxxxx add ln -sfn $(pwd) /home/jinn/DP091venv/bin/python  xxxxx Error source
--- OK by ln but new Error
--- Error: /DP091venv/bin/"python3" is bad now
--- delete (DP091venv)

230329a: OK. ??? UI: PC: DP091aL: 230309: Run "Replay" "USA Data"
--- move /openpilot/selfdrive/debug/uiview.py to /openpilot
  (DP091venv) jinn@Liu:~/openpilot$ python uiview.py
  (DP091venv) jinn@Liu:~/openpilot/tools/replay$ ./replay --demo
--- Why not jinn@Liu:~/openpilot/selfdrive/debug$ python uiview.py ???

230329: OK scons PC: "DP091aL" using ".pyenv_DP091L", "DP091venv"
--- Continued from 230327.
  (DP091venv) jinn@Liu:~/openpilot$ scons -u -j$(nproc)
  Ch16. - Ch21. do similarly
--- all done finished

230327: OK. PC: Change scons "DP091aL"
--- Change DP091a from DP github to DP091aL = PC/openpilot. Continued from Ch5. 230324.
  (DP091venv) jinn@Liu:~/openpilot$ scons -u -j$(nproc)
  Ch6. do 'cereal/SConscript' Line 361 in /openpilot/SConstruct
  Ch7. - Ch15. do similarly
--- all done

230324: OK. PC: Change scons "DP091L", get "DP091a"
--- Change DP091 from C2 to DP091L == PC/openpilot. See 230319 Read ToDo.
  (DP091venv) jinn@Liu:~/openpilot$ tools/ubuntu_setup.sh
  (DP091venv) jinn@Liu:~/openpilot$ scons -u -j$(nproc)
  Ch1. Change PC/openpilot to PC/OP091 and PC/DP091 to PC/openpilot == PC/DP091L (L: long, S: short)
  Ch2. Do "venv", "ubuntu_setup.sh" in Step (B) in InstallOP.docx for DP091L
  Ch3. Do 'common/SConscript' Line 349 in /openpilot/SConstruct
  Ch4. Change /openpilot to /DP091L and /OP091 to /openpilot
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
  Ch5. Change all /DP091L back to /openpilot. Do SConscript(['cereal/SConscript'])
  -- get 'dp.capnp.h' from https://github.com/dragonpilot-community/dragonpilot
  -- !!! stop scons DP091L. use DP091a downloaded from this github.
--- get "DP091a" from https://github.com/dragonpilot-community/dragonpilot

===== Details of Coding Notes =====

230417: OK Ctr 1: CBQ 1: PC+C2+body: "controlsd.py", "Speed"
--- Redo 230302, 230118, 230213
--- try 1
--- reboot C2
  root@localhost:/data/openpilot/tools/joystick$ python joystickd.py --keyboard
--- ignize C2
--- NG: no "Joystick Mode"
--- try 2
--- reboot C2
  root@localhost:/data/openpilot/tools/joystick$ echo -n "1" > /data/params/d/JoystickDebugMode
  root@localhost:/data/openpilot/cereal/messaging$ ./bridge 10.0.0.4 testJoystick
  root@localhost:/data/openpilot/tools/joystick$ export ZMQ=1
  root@localhost:/data/openpilot/tools/joystick$ python joystickd.py --keyboard
--- ignize C2
--- NG C2 only responded to "steer" (AD) but not "gb" (WS) and no "Joystick Mode"
--- try 3
--- reboot C2
  root@localhost:/data/openpilot/tools/joystick$ python web.py
--- browser =>
  http://10.0.0.4:5000/
--- OK "Green Ball"
--- ignize C2
--- too fast warning sound
--- Change Line 47 /body/carcontroller.py:
  speed_desired = CC.actuators.accel / 20.  # JLL NG
--- reboot C2
--- NG "Speed"
--- try 4
--- Change Line 638 /controls/controlsd.py:
          actuators.accel = 1.5*clip(self.sm['testJoystick'].axes[0], -1, 1)  # JLL OK
--- OK. Solved Ctr 1 "Speed"
--- NG: no "Joystick Mode"
--- CBQ 1: body vibrates a lot in calibration due to "PIDController",
           body cannot go straight line ???

230417a: NG. Ctr: iPhone+PC+C2+body: "QR code"
--- A. iPhone: scan QR code on C2
  > Mapbox PUBLIC TOKEN (sk.sxxxxxx6) >
  > http://10.0.0.4:8082 > open link > Search a place (DRAGONPILOT)
--- this is for Mapbox navigation NOT for ssh to C2
--- Hosts - Termius support: import the key from Keychain > Import a key > iOS
  > Choose Keychain in the app's main menu.
--- FileZilla: Key file: /home/jinn/OP/OPkeys/c2key089.ppk
  > upload c2key089.ppk to Google Drive (GD) > iPhone: download c2key089.ppk from GD
--- Where is SSH passphrase stored?
    You will be prompted for a passphrase which is used to encrypt the private key.
    By default, the private key is stored in ~/. ssh/id_rsa and the public key is
    stored in ~/. ssh/id_rsa.pub
  root@localhost:~$ pwd
    /data/data/com.termux/files/home
  root@localhost:~$ ls
    Pipfile  Pipfile.lock  install.sh  setup_keys
--- B. iPhone: Termius ssh
  iPhone > AppStore > insall Termius App (for ssh) > password: sxxxmxxxkx
  > Termius menu: Hosts, Terminal, ..., Keychain, ...  > Keychain > Tap + >
  > tap Import key > tap c2key089.ppk > tap Save > Termius menu: tap Hosts
  > Tap + > New Host > Alias: C2 > Hostname (IP address): 10.0.0.4 > Port 8022
  > Username: root > Key: c2key089.ppk > tap Save
--- ssh: Edit c2key089.ppk > Passphrase: sxxxxxx6 > tap Save
  > Hosts > C2 > passphrase: sxxxxxx6 > tap Connect > Authentication failed (publickey) >
--- NG: Termius ssh

230414: UI: PC: DP091L: Change Run "controlsdJLL.py", "controlsd091JLL.py"
--- see 230201
  (sconsvenv) jinn@Liu:~/openpilot$ python controlsdJLL.py
    self.branch = get_short_branch("")
      @cache
      def get_short_branch(default: Optional[str] = None) -> Optional[str]:
        - functools — Higher-order functions and operations on callable objects
        - Caching in Python Using the LRU Cache Strategy
--- see 230213 ToDo Read

230413: OK UI: PC: DP091L: redo 230306
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
--- Error
    File "/home/jinn/openpilot/selfdrive/car/honda/interface.py", line 58, in _get_params
      if int(params.get("dp_atl").decode('utf-8')) == 1:
    File "common/params_pyx.pyx", line 56, in common.params_pyx.Params.get
      cdef string k = self.check_key(key)
    File "common/params_pyx.pyx", line 52, in common.params_pyx.Params.check_key
      raise UnknownKeyName(key)
    common.params_pyx.UnknownKeyName: b'dp_atl'
--- rename /openpilot/selfdrive/car to ../car_DP091
--- move /OP091/selfdrive/car to ../car
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python cycle_alertsJLL.py

--- OK: "Joystick Mode Gas: -30%,Steer: 70%"

230412: OK UI 0: PC: DP091L: "uiviewJLL.py", "bodyJLL.py", 'controlsState'
--- redo 230304
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
    /home/jinn/openpilot/cereal/messaging/__init__.py:258: UserWarning: This message
    has already been written once. Be very careful that you're not setting
    Text/Struct/List fields more than once, since that will cause memory leaks
    (both in memory and in the serialized data). You can disable this warning by
    calling the `clear_write_flag` method of this object after every write.
      dat = dat.to_bytes()
--- UserWarning is due to pm.send() in while True:
  pm = messaging.PubMaster(['deviceState', 'pandaStates'])
--- OK: 'ui' > few seconds > "openpilot Unavailable Waiting for controls to start"
  pm = messaging.PubMaster(['controlsState', 'deviceState', 'pandaStates'])
--- Solved UI 0: 'controlsState'
--- Terminal 2
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python bodyJLL.py
--- OK: 'ui' > carParams.notCar > "Body Face" > carState.charging > carState.fuelGauge
--- Terminal 3
  (sconsvenv) jinn@Liu:~/openpilot$ python joystickdJLL.py  --keyboard
--- No change

230411: ===== Talk for Saber's Blog =====
--- Introduction
  ModelB5 for Self-Driving Cars: https://github.com/JinnAIGroup/B5
  C2: B5 and Comma Body Notes: https://docs.google.com/document/d/124QjOUCyWNc1lrmbkbvUpVL4ssD56lMl/edit
--- How to do OP coding
  1. Learn Python
  2. Choose a code editor (Visual Studio)
  3. Run, Change, and Read an OP code on PC and PC+C2
  4. Do a UI, Model, or Control Project at a time
  5. Take notes: copy commands, code changes, and error messages etc.
--- OP/DP Versions:
  1. C2 DP091:   Version 0.9.1 (2022-12-XX) downloaded on 2023-01-17
  2. PC OP091:   Version 0.9.1 (2022-12-XX) downloaded on 2023-01-19
  3. PC DP091a:  Version 0.9.1 (2023-02-23) downloaded on 2023-03-27
  4. PC DP091aL: Long DP091a changed by JLL
--- Demo
OP: fcamera.hevc, DP: 2023-03-18_02-30-42.mp4
230118: OK Install: DP091 = DPbeta2 on C2+body/car
230311: OK UI: PC: Change "cameraJLL.cc", scons Change Run "./replayJLL --data_dir
230317: OK NG UI: PC+C2: Run "uiview.py" ("Green Path" on C2. No "fcamera.hevc")
230318: OK Run Car+C2: Use "Enable On-Road Dashcam", "啟用行車記錄功能"
230319: ??? UI: PC+C2: Run Read "tmux at"
  root@localhost:/data/openpilot$ tmux at
    camerad clocksd modeld sensord ui controlsd dmonitoringd pandad paramsd plannerd
    radard dpmonitoringd mapd systemd ...  (no "loggerd" ???)
230329: OK scons PC: "DP091aL" using ".pyenv_DP091L", "DP091venv"
230406: OK. ??? UI: PC: DP091L: Change Run "launch_openpilotJLL.sh" from 230330
  (sconsvenv) jinn@Liu:~/openpilot$ bash launch_openpilotJLL.sh
230410b: OK. Redo 230311 on DP091aL
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
  (sconsvenv) jinn@Liu:~/openpilot/tools/replay$
    ./replayJLL --data_dir dataC "8bfda98c9c9e4291|2020-05-11--03-00-57--61"
===== Talk End =====

230410b: OK. ??? Redo 230311 on DP091aL
--- move /OP091/../*JLL.* to /openpilot/../*JLL.*
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
--- ??? ./ui: 11: [: 0: unexpected operator
  (sconsvenv) jinn@Liu:~/openpilot/tools/replay$
    ./replayJLL --data_dir dataC "8bfda98c9c9e4291|2020-05-11--03-00-57--61"

230410a: OK Install: "Kazam"
  A. do How to Switch Between Wayland and Xorg in Ubuntu
  B. do Record Screen in Ubuntu Linux With Kazam [Beginner's Guide]
  C. If no sound, Fix Kazam No Sound on X11 or XORG
      jinn@Liu:/usr/lib/python3/dist-packages/kazam/pulseaudio$ sudo chmod 777 pulseaudio.py
      jinn@Liu:/usr/lib/python3/dist-packages/kazam/pulseaudio$ gedit pulseaudio.py
      -- replace time.clock() with time.perf_counter() in all 8 places
      -- set Kazam file date: File > Preferences > Sreencast > Finename prefix > Kazam_2023-04-10
      -- read How to Record Your Desktop with Audio in Ubuntu 22.04 LTS
              kazam no sound and no rectangular red border during
  D. Kazam recording steps: Click Kazam > Preferences > Sceencast > Fielename prefix
     > Kazam > Click Capture > See "Red" Camara on the Top Screen on Ubuntu >
     Click "Pause" > "Finish" > Done or Record again

230406: OK. ??? UI: PC: DP091L: Change Run "launch_openpilotJLL.sh" from 230330
  (sconsvenv) jinn@Liu:~/openpilot$ bash launch_openpilotJLL.sh
    cd selfdrive/manager && ./manager.py
--- Error 1
  File "./manager.py", line 10, in <module>
  ModuleNotFoundError: No module named 'cereal'
--- add
  ln -sfn $(pwd) /home/jinn/sconsvenv/bin
    # creat the source (target) directory $(pwd) as a symbolic link in /bin
  export PYTHONPATH="$PWD"
    # Set PYTHONPATH to the working directory PWD
--- get
  /home/jinn/sconsvenv/bin/openpilot
    Type: Link to Folder (inode/directory)
    Link target: /home/jinn/openpilot
  (sconsvenv) jinn@Liu:~/openpilot$ which python
    /home/jinn/sconsvenv/bin/python
  (sconsvenv) jinn@Liu:~/openpilot$ echo $PWD
    /home/jinn/openpilot
  (sconsvenv) jinn@Liu:~/openpilot$ echo $PYTHONPATH
    /home/jinn/openpilot
  (sconsvenv) jinn@Liu:~/openpilot$ echo $NOBOARD
    -- nothing
  (sconsvenv) jinn@Liu:~/openpilot$ echo $BLOCK
    -- nothing
    Environment variables (PYTHONPATH etc.) are global system variables accessible
      by all the processes/users running under the Operating System (OS),
      such as Windows, macOS and Linux.
  (sconsvenv) jinn@Liu:~/openpilot$ export BLOCK="camerad,loggerd,encoderd,micd"
  (sconsvenv) jinn@Liu:~/openpilot$ env | grep BLOCK
    BLOCK=camerad,loggerd,encoderd,micd
  (sconsvenv) jinn@Liu:~/openpilot$ echo $BLOCK
    camerad,loggerd,encoderd,micd
  (sconsvenv) jinn@Liu:~/openpilot$ deactivate
  jinn@Liu:~/openpilot$ . ~/sconsvenv/bin/activate
  (sconsvenv) jinn@Liu:~/openpilot$ echo $BLOCK
    camerad,loggerd,encoderd,micd
    -- why nothing when export by launch_openpilotJLL.sh ???
      A: export exports the variable assignment to child processes of the shell
         in which the export command was ran. The command-line environment
         is the parent of the script's shell, so it does not see the variable assignment.
      (sconsvenv) jinn@Liu:~/openpilot$ source ./launch_openpilotJLL.sh
      (sconsvenv) jinn@Liu:~/openpilot/selfdrive/manager$
      (sconsvenv) jinn@Liu:~/openpilot/selfdrive/manager$ echo $NOBOARD
        1
    -- add
      echo "#--- FINGERPRINT = " $FINGERPRINT
    -- now all env variables are accesible to all processes in the Bash shell
      (sconsvenv) jinn@Liu:~/openpilot$ echo $FINGERPRINT
        HONDA CIVIC 2016
      (sconsvenv) jinn@Liu:~/openpilot$ echo $SKIP_FW_QUERY
        1
      (sconsvenv) jinn@Liu:~/openpilot$ echo $NOBOARD
        1
--- Error 2
  File "./manager.py", line 20, in <module>
  File "common/params_pyx.pyx", line 52, in common.params_pyx.Params.check_key
    common.params_pyx.UnknownKeyName: b'dp_api_custom'
--- the ln is permanent but need to export PYTHONPATH="$PWD"
  (sconsvenv) jinn@Liu:~/openpilot$ deactivate
  (sconsvenv) jinn@Liu:~/openpilot/selfdrive/manager$ echo $PWD
    /home/jinn/openpilot/selfdrive/manager
  (sconsvenv) jinn@Liu:~/openpilot/selfdrive/manager$ ./manager.py
--- Error 1
--- set
  #ln -sfn $(pwd) /home/jinn/sconsvenv/bin
  (sconsvenv) jinn@Liu:~/openpilot$ bash launch_openpilotJLL.sh
  (sconsvenv) jinn@Liu:~/openpilot$
--- Error 2
  (sconsvenv) jinn@Liu:~/openpilot$ export PYTHONPATH="$PWD"
  (sconsvenv) jinn@Liu:~/openpilot$ cd selfdrive/manager && ./manager.py
  (sconsvenv) jinn@Liu:~/openpilot/selfdrive/manager$
--- Error 2
--- these two are NOT the same
--- Conclusion: use $ bash launch_openpilotJLL.sh w/o echo ...
--- fix Error 2
    common.params_pyx.Params.check_key
      params: remove params_pxd.pxd (#22723) deanlee committed on Oct 29, 2021
        https://github.com/commaai/openpilot/search?q=_pxd&type=commits
    common/SConscript > envCython.Program('params_pyx.so', 'params_pyx.pyx', >
      params_pyx.cpp > params_pyx.o
    common.params_pyx.UnknownKeyName: b'dp_api_custom'
--- Pipe Cython: params.h > params.cc > params.o > params_pyx.pyx > params_pyx.so >
    params_pyx.cpp > params_pyx.o > params.py
--- Change common/api/__init__.py Line 10 from OP091
  (sconsvenv) jinn@Liu:~/openpilot$ bash launch_openpilotJLL.sh
--- Error 3
    File "./manager.py", line 20, in <module>
    File "/home/jinn/openpilot/selfdrive/controls/lib/events.py", line 597, in <module>
    AttributeError: '_EnumModule' object has no attribute 'speedLimitActive'
--- Change /selfdrive/controls/lib/events.py to events_DP091.py
--- Copy /OP091/../events.py to /openpilot/../events.py
--- Error 4
    Manager failed to start
    Traceback (most recent call last):
      File "./manager.py", line 227, in <module>
      File "./manager.py", line 61, in manager_init
      File "common/params_pyx.pyx", line 52, in common.params_pyx.Params.check_key
    common.params_pyx.UnknownKeyName: b'ShowDebugUI'
--- Change /selfdrive/manager/manager.py to manager_DP091.py
--- Copy /OP091/../manager.py to ../manager.py
--- Error 5
    selfdrive/loggerd/bootlog.cc: bootlog to /home/jinn/.comma/media/0/realdata/boot/2023-04-08--14-30-25
    missing public key: /home/jinn/.comma/persist/comma/id_rsa.pub
    ./ui: 11: [: 0: unexpected operator
    Manager failed to start
    File "./manager.py", line 101, in manager_prepare
    File "/home/jinn/openpilot/selfdrive/manager/process.py", line 233, in prepare
    File "/home/jinn/openpilot/selfdrive/controls/plannerd.py", line 6, in <module>
    File "/home/jinn/openpilot/selfdrive/controls/lib/longitudinal_planner.py", line 17, in <module>
    File "/home/jinn/openpilot/selfdrive/controls/lib/vision_turn_controller.py", line 54, in <module>
    AttributeError: '_StructModule' object has no attribute 'VisionTurnControllerState'
--- Change /selfdrive/manager/manager.py Line 32
--- Change /selfdrive/manager/process.py to process_DP091.py
--- Copy /OP091/../process.py to ../process.py
--- Error 5
--- Change /selfdrive/manager/process_config.py to process_config_DP091.py
--- Copy /OP091/../process_config.py to ../process_config.py
--- Error 5
--- Change /selfdrive/controls/lib/longitudinal_planner.py to longitudinal_planner_DP091.py
--- Copy /OP091/../longitudinal_planner.py to ../longitudinal_planner.py
--- Error 6
    File "/home/jinn/openpilot/selfdrive/controls/plannerd.py", line 10, in <module>
    File "/home/jinn/openpilot/selfdrive/dragonpilot/controls_0813/lib/lateral_planner.py", line 5, in <m
--- Change /selfdrive/controls/lib/lateral_planner.py to lateral_planner_DP091.py
--- Copy /OP091/../lateral_planner.py to ../lateral_planner.py
--- Change /selfdrive/controls/plannerd.py to plannerd_DP091.py
--- Copy /OP091/../plannerd.py to ../plannerd.py
--- Error 7
    missing public key: /home/jinn/.comma/persist/comma/id_rsa.pub
    ./ui: 11: [: 0: unexpected operator
    Manager failed to start
    Traceback (most recent call last):
      File "./manager.py", line 204, in <module>
        main()
      File "./manager.py", line 172, in main
        manager_prepare()
      File "./manager.py", line 101, in manager_prepare
        p.prepare()
      File "/home/jinn/openpilot/selfdrive/manager/process.py", line 233, in prepare
      File "/home/jinn/openpilot/selfdrive/controls/radard.py", line 11, in <module>
      OSError: cannot load library '/home/jinn/openpilot/selfdrive/controls/lib/cluster/libfastcluster.so'
        No such file or directory.
--- Change /selfdrive/controls/radard.py to radard_DP091.py
--- Copy /OP091/../radard.py to ../radard.py
--- Terminal 1
  (sconsvenv) jinn@Liu:~/openpilot$ bash launch_openpilotJLL.sh
    ...
    common.params_pyx.UnknownKeyName: b'dp_auto_shutdown'
    logmessaged ui soundd deleter thermald statsd
    ...
--- OK: ui
--- Terminal 2
  (sconsvenv) jinn@Liu:~/openpilot$ ps au
    USER         PID %CPU %MEM    VSZ   RSS TTY      STAT START   TIME COMMAND
    jinn        1332  0.0  0.0 164316  6520 tty2     Ssl+ Apr07   0:00 /usr/lib/gdm3/gdm-x-s
    jinn        1334  1.3  0.6 630120 105428 tty2    Sl+  Apr07  24:36 /usr/lib/xorg/Xorg vt
    jinn        1347  0.0  0.0 188520 13704 tty2     Sl+  Apr07   0:00 /usr/libexec/gnome-se
    jinn       20039  0.0  0.0  11308  5376 pts/0    Ss   Apr07   0:00 bash
    jinn       57282  0.0  0.0   9928  3468 pts/0    S+   19:34   0:00 bash launch_openpilot
    jinn       57285  0.8  0.3 403368 57092 pts/0    S+   19:34   0:00 python3 ./manager.py
    jinn       57298  0.8  0.6 594428 103400 pts/1   Ssl+ 19:34   0:00 python3 ./manager.py
    jinn       57308  1.1  0.7 1012592 114920 pts/1  Sl+  19:34   0:01 ./_ui
    jinn       57318  0.0  0.5 614648 84544 pts/1    Sl+  19:34   0:00 system.logmessaged
    jinn       57321  0.2  0.1 767376 28284 pts/1    Sl+  19:34   0:00 ./_soundd
    jinn       57322  0.2  0.4 594160 80900 pts/1    S+   19:34   0:00 selfdrive.loggerd.del
    jinn       57334  0.0  0.5 604404 84648 pts/1    Sl+  19:34   0:00 selfdrive.statsd
--- Pipe: launch_openpilotJLL.sh > ./manager.py > logmessaged, ui, soundd, deleter,
    thermald, statsd (no "loggerd" as 230319 ???)
--- OK.

230404: OK scons PC: "DP091L = DP091aL" using "/.pyenv_OP091", "/sconsvenv_OP091"
--- Change /openpilot to /OP091 and /DP091L to /openpilot
--- copy .pyenv_OP091 to .pyenv
  jinn@Liu:~$ cp -avr .pyenv_OP091 .pyenv
  jinn@Liu:~$ cp -avr sconsvenv sconsvenv_OP091
--- Backups: .pyenv_OP091, sconsvenv_OP091
  jinn@Liu:~/openpilot$ . ~/sconsvenv/bin/activate
  (sconsvenv) jinn@Liu:~/openpilot$ python --version
    Python 3.8.2
  (sconsvenv) jinn@Liu:~/openpilot$ pyenv versions
    * 3.8.10 (set by /home/jinn/openpilot/.python-version)
  (sconsvenv) jinn@Liu:~/openpilot$ which python
    /home/jinn/sconsvenv/bin/python
  (sconsvenv) jinn@Liu:~/openpilot$ tools/ubuntu_setup.sh
----   OPENPILOT SETUP DONE   ----
--- OK
  (sconsvenv) jinn@Liu:~/openpilot$ scons -u -j$(nproc)
    scons: done building targets.
--- OK
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
  (sconsvenv) jinn@Liu:~/openpilot/tools/replay$
    ./replayJLL --data_dir dataC "8bfda98c9c9e4291|2020-05-11--03-00-57--61"
--- OK
--- delete .pyenv_DP091L, DP091venv

230404a: OK UI: PC: OP091: Redo 230324 Ch4.
--- Change /openpilot to /DP091L and /OP091 to /openpilot
--- rename .pyenv to .pyenv_DP091L and .pyenv_OP091 to .pyenv
--- check /.pyenv/versions/3.8.2
  jinn@Liu:~$ gedit ~/.bashrc
  jinn@Liu:~/openpilot$ . ~/sconsvenv/bin/activate
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
  (sconsvenv) jinn@Liu:~/openpilot/tools/replay$
    ./replayJLL --data_dir dataC "8bfda98c9c9e4291|2020-05-11--03-00-57--61"
--- OK.
  (sconsvenv) jinn@Liu:~/openpilot$ which pythonpath
  (sconsvenv) jinn@Liu:~/openpilot$ which python
  /home/jinn/sconsvenv/bin/python
  (sconsvenv) jinn@Liu:~/openpilot$ pyenv versions
    system
    3.6.5
    3.6.5/envs/snpe
  * 3.8.10 (set by /home/jinn/openpilot/.python-version)
  (sconsvenv) jinn@Liu:~/openpilot$ python --version
    Python 3.8.2
  jinn@Liu:~$ python --version
  Python 3.8.2

230330: NG. UI: PC: DP091aL: Do 230329a ??? by "launch_openpilotJLL.sh"
--- move /uiview.py to /openpilot/selfdrive/debug
  (DP091venv) jinn@Liu:~/openpilot/selfdrive/manager$ ./manager.py
--- Error
    ModuleNotFoundError: No module named 'cereal'
--- need to use "launch_openpilot.sh"
  /openpilot/tools/sim/launch_openpilot.sh
    DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
      -- it gives the absolute path to the script ${} quotes for variables.
         BASH_SOURCE is an array of source file pathnames. This variable is available
           when a script is being sourced or executed by bash.
           It contains nothing in an interactive session.
         $(command) will call the command and return the stdout in a variable
         (DP091venv) jinn@Liu:~$ dirname /home/jinn/openpilot/tools/JLL
           /home/jinn/openpilot/tools
         cd changes the current directory.
         pwd gives the current path.
         /dev/null is a special device (file) that, if written to it, discards and
           if read from, reads null.
    cd ../../selfdrive/manager && exec ./manager.py
  /openpilot/launch_openpilot.sh
    exec ./launch_chffrplus.sh
  (DP091venv) jinn@Liu:~/openpilot$ ln -s launch_openpilotJLL.sh JLL.sh
  (DP091venv) jinn@Liu:~/openpilot$ ls -l JLL.sh
    lrwxrwxrwx 1 jinn jinn 22 Mar 31 11:10 JLL.sh -> launch_openpilotJLL.sh
  (DP091venv) jinn@Liu:~/openpilot$ rm JLL.sh
  (DP091venv) jinn@Liu:~/openpilot$ which python
    /home/jinn/DP091venv/bin/python
xxxxx add ln -sfn $(pwd) /home/jinn/DP091venv/bin/python  xxxxx Error source
      Linux ln – How to Create a Symbolic Link
        -s soft links
  (DP091venv) jinn@Liu:~/openpilot$ bash launch_openpilotJLL.sh
    ModuleNotFoundError: No module named 'capnp'
--- OK by ln but new Error
  (DP091venv) jinn@Liu:~/openpilot$ ls -l /home/jinn/DP091venv/bin/python
    lrwxrwxrwx 1 jinn jinn 20 Mar 31 11:30 /home/jinn/DP091venv/bin/python -> /home/jinn/openpilot
  (DP091venv) jinn@Liu:~/openpilot$ unlink /home/jinn/DP091venv/bin/python
  (DP091venv) jinn@Liu:~/openpilot$ which python
    /home/jinn/.pyenv/shims/python
xxxxx Error: should be /home/jinn/DP091venv/bin/python
xxxxx Link (broken) (inode/symlink): /home/jinn/DP091venv/bin/python3
  (DP091venv) jinn@Liu:~/openpilot$ poetry shell
    bash: /home/jinn/DP091venv/bin/poetry: /home/jinn/DP091venv/bin/python: bad interpreter: No such file or directory
--- Error: /DP091venv/bin/"python3" is bad now
--- NG.

230329a: OK. ??? UI: PC: DP091aL: 230309: Run "Replay" "USA Data"
--- move /openpilot/selfdrive/debug/uiview.py to /openpilot
  (DP091venv) jinn@Liu:~/openpilot$ python uiview.py
  (DP091venv) jinn@Liu:~/openpilot/tools/replay$ ./replay --demo
--- clean up "comma_download_cache"
  jinn@Liu:~$ sudo passwd root
  jinn@Liu:~$ su root
  root@Liu:/tmp/comma_download_cache# ls -sh
  root@Liu:/tmp# rm -rf comma_download_cache
  root@Liu:~# exit
--- OK but CAN 19 =>
  root@localhost:/data/openpilot/selfdrive/debug$ python uiview.py
  => OK in C2 but not in PC =>
--- Why not jinn@Liu:~/openpilot/selfdrive/debug$ python uiview.py ???

230329: OK scons PC: "DP091aL" using ".pyenv_DP091L", "DP091venv"
--- Continued from 230327.
  Ch16. do 'selfdrive/modeld/SConscript' Line 424 in /openpilot/SConstruct
  --- add
    /home/jinn/openpilot/selfdrive/modeld/SConscript
    /home/jinn/openpilot/selfdrive/modeld/dmonitoringmodeld.cc
    /home/jinn/openpilot/selfdrive/modeld/modeld.cc
    /home/jinn/openpilot/selfdrive/modeld/navmodeld.cc
    /home/jinn/openpilot/selfdrive/modeld/runners/onnxmodel.cc
    /home/jinn/openpilot/selfdrive/modeld/runners/onnxmodel.h
    /home/jinn/openpilot/selfdrive/modeld/runners/snpemodel.cc
    /home/jinn/openpilot/selfdrive/modeld/runners/thneedmodel.cc
  --- rename /openpilot/panda/ to /panda_DP091a
  --- add /OP091/panda to /openpilot
    /home/jinn/openpilot/selfdrive/modeld/models/commonmodel.cc
    /home/jinn/openpilot/selfdrive/modeld/models/dmonitoring.cc
    /home/jinn/openpilot/selfdrive/modeld/models/driving.cc
    /home/jinn/openpilot/selfdrive/modeld/models/nav.cc
    /home/jinn/openpilot/third_party/cluster
    /home/jinn/openpilot/selfdrive/modeld/transforms/loadyuv.cc
    /home/jinn/openpilot/selfdrive/modeld/transforms/transform.cc
  --- OK. done

  Ch17. do Lines 426 427 429 in /openpilot/SConstruct
  --- add
    /home/jinn/openpilot/selfdrive/controls/lib/lateral_mpc_lib/SConscript
    /home/jinn/openpilot/third_party/acados/acados_template
    /home/jinn/openpilot/selfdrive/controls/lib/longitudinal_mpc_lib/SConscript
    /home/jinn/openpilot/selfdrive/boardd/boardd.cc
    /home/jinn/openpilot/selfdrive/boardd/can_list_to_can_capnp.cc
    /home/jinn/openpilot/selfdrive/boardd/main.cc
    /home/jinn/openpilot/selfdrive/boardd/panda.cc
    /home/jinn/openpilot/selfdrive/boardd/panda_comms.cc
    /home/jinn/openpilot/selfdrive/boardd/spi.cc

  Ch18. do 'selfdrive/loggerd/SConscript' Line 431 in /openpilot/SConstruct
  --- add
    /home/jinn/openpilot/selfdrive/loggerd/bootlog.cc
    /home/jinn/openpilot/selfdrive/loggerd/encoderd.cc
    /home/jinn/openpilot/selfdrive/loggerd/logger.cc
    /home/jinn/openpilot/selfdrive/loggerd/loggerd.cc
    /home/jinn/openpilot/selfdrive/loggerd/video_writer.cc
    /home/jinn/openpilot/selfdrive/loggerd/encoder/encoder.cc
    /home/jinn/openpilot/selfdrive/loggerd/encoder/ffmpeg_encoder.cc
    /home/jinn/openpilot/selfdrive/loggerd/encoder/v4l_encoder.cc

  Ch19. do Lines 433 434 in /openpilot/SConstruct
  --- add
    /home/jinn/openpilot/selfdrive/locationd/SConscript
    /home/jinn/openpilot/selfdrive/locationd/liblocationd.cc
    /home/jinn/openpilot/selfdrive/locationd/locationd.cc
    /home/jinn/openpilot/selfdrive/locationd/ubloxd.cc
    /home/jinn/openpilot/selfdrive/locationd/ublox_msg.cc
    /home/jinn/openpilot/selfdrive/locationd/models/live_kf.cc
    /home/jinn/openpilot/selfdrive/locationd/ublox_msg.h
  --- Change
    /home/jinn/openpilot/selfdrive/locationd/ublox_msg_DP091a.h
  --- add
    /home/jinn/openpilot/selfdrive/sensord/SConscript
    /home/jinn/openpilot/selfdrive/sensord/sensors_qcom2.cc
    /home/jinn/openpilot/selfdrive/sensord/sensors/bmx055_accel.cc
    /home/jinn/openpilot/selfdrive/sensord/sensors/bmx055_gyro.cc
    /home/jinn/openpilot/selfdrive/sensord/sensors/bmx055_magn.cc
    /home/jinn/openpilot/selfdrive/sensord/sensors/bmx055_temp.cc
    /home/jinn/openpilot/selfdrive/sensord/sensors/file_sensor.cc
    /home/jinn/openpilot/selfdrive/sensord/sensors/i2c_sensor.cc
    /home/jinn/openpilot/selfdrive/sensord/sensors/light_sensor.cc
    /home/jinn/openpilot/selfdrive/sensord/sensors/lsm6ds3_accel.cc
    /home/jinn/openpilot/selfdrive/sensord/sensors/lsm6ds3_gyro.cc
    /home/jinn/openpilot/selfdrive/sensord/sensors/lsm6ds3_temp.cc
    /home/jinn/openpilot/selfdrive/sensord/sensors/mmc5603nj_magn.cc

  Ch20. do 'selfdrive/ui/SConscript' Line 435 in /openpilot/SConstruct
  --- Change
    /home/jinn/openpilot/selfdrive/ui/qt_DP091a
  --- add
    /home/jinn/openpilot/selfdrive/ui/SConscript
    /home/jinn/openpilot/selfdrive/ui/main.cc
    /home/jinn/openpilot/selfdrive/ui/moc_ui.cc
    /home/jinn/openpilot/selfdrive/ui/mui.cc
    /home/jinn/openpilot/selfdrive/ui/ui.cc
    /home/jinn/openpilot/selfdrive/ui/ui.h
    /home/jinn/openpilot/selfdrive/ui/watch3.cc
    /home/jinn/openpilot/selfdrive/ui/installer
    /home/jinn/openpilot/selfdrive/ui/qt
    /home/jinn/openpilot/selfdrive/ui/soundd/main.cc
    /home/jinn/openpilot/selfdrive/ui/soundd/sound.cc
    /home/jinn/openpilot/selfdrive/ui/soundd/sound.h
    /home/jinn/openpilot/third_party/mapbox-gl-native-qt/x86_64
  --- OK. done

  Ch21. do Lines 436 439 443 447 in /openpilot/SConstruct
  --- Change
    /home/jinn/openpilot/selfdrive/navd_DP091a
  --- add
    /home/jinn/openpilot/selfdrive/navd
    /home/jinn/openpilot/tools/cabana
  --- Warnings
    scons: Reading SConscript files ...
    fatal: Needed a single revision
    Git commit hash for gitversion.h: 00000000
    fatal: not a git repository: /home/jinn/openpilot/panda/../.git/modules/panda
    scons: warning: Two different environments were specified for target /tmp/scons_cache/moc_files/moc_map_renderer.cc,
    	but they appear to have the same action: $QT_MOC $QT_MOCFROMHFLAGS -o ${TARGETS[0]} $SOURCE
    File "/home/jinn/openpilot/selfdrive/navd/SConscript", line 21, in <module>
    scons: done reading SConscript files.
    scons: Building targets ...
    generate_dbc_json(["tools/cabana/generate_dbc_json"], [])
    tools/cabana/generate_dbc_json.py --out tools/cabana/car_fingerprint_to_dbc.json
    terminate called after throwing an instance of 'std::runtime_error'
      what():  Failed to ensure params path, errno=13
    Aborted (core dumped)
    scons: *** Error 134
    scons: done building targets.
  --- OK. all done finished

230327: OK. PC: Change scons "DP091aL"
--- Change DP091a from DP github to DP091aL = PC/openpilot. Continued from Ch5. 230324.

  Ch6. do 'cereal/SConscript' Line 361 in /openpilot/SConstruct
  --- run openpilot/SConstruct to add missing files
    (DP091venv) jinn@Liu:~/openpilot$ scons -u -j$(nproc)
      `cereal/messaging/messaging.cc' not found
      `cereal/messaging/impl_msgq.cc' not found
      `cereal/messaging/impl_zmq.cc' not found
      `cereal/messaging/msgq.cc' not found
      `cereal/messaging/socketmaster.cc' not found
      `cereal/visionipc/ipc.cc' not found
      `cereal/visionipc/visionipc_server.cc' not found
      `cereal/visionipc/visionipc_client.cc' not found
      `cereal/visionipc/visionbuf.cc' not found
      `cereal/visionipc/visionbuf_cl.cc' not found
      `cereal/messaging/bridge.cc' not found
    -- Error
      In file included from cereal/gen/cpp/log.capnp.c++:4:
      In file included from cereal/gen/cpp/log.capnp.h:14:
      cereal/gen/cpp/dp.capnp.h:10:2: error: "Version mismatch between generated code
      and library headers.  You must use the same version of the Cap'n Proto compiler
      and library."

  Ch7. do 'cereal/SConscript' Line 361 with cereal/SConstruct
  --- Change Line 11 in /SConscript to
    schema_files = ['log.capnp', 'car.capnp', 'legacy.capnp', 'dp.capnp']
  --- rename cpp to cereal/gen/cpp_old ??? No.
  --- cereal$ scons or openpilot$ scons -u -j$(nproc) ???
     -u, --up, --search-up
       Walks up the directory structure until an SConstruct is
       found, and uses that as the top of the directory tree. If no
       -u, only targets at or below the current directory will be built.
     -j N, --jobs=N
       Specifies the maximum number of comcurrent jobs (commands) to run.
     $(nproc) > jinn@Liu:~$ (nproc) > 8
  --- 2 SConstructs in the tree now:
    openpilot/SConstruct, openpilot/cereal/SConstruct
  --- try openpilot$ scons -u -j$(nproc)
  --- OK. all files in /gen/cpp have been updated.
    `common/params.cc' not found
    `common/statlog.cc' not found
    `common/swaglog.cc' not found
    `common/util.cc' not found
    `common/i2c.cc' not found
    `common/watchdog.cc' not found
  --- Error
    clang++ -o cereal/messaging/bridge -Wl,--as-needed ...
      -rpath=/home/jinn/openpilot/third_party/acados/x86_64/lib
      -rpath=/home/jinn/openpilot/third_party/snpe/x86_64-linux-clang
      -rpath=/home/jinn/openpilot/cereal
      -rpath=/home/jinn/openpilot/common
      cereal/libmessaging.a -lzmq common/libcommon.a -ljson11 ...
    /usr/bin/ld: cannot find -ljson11
  --- add
    /openpilot/third_party/snpe/x86_64
    /openpilot/third_party/snpe/x86_64-linux-clang
  --- oops these 2 "moved" from /OP092 to /openpilot were "deleted"
    /OP091/cereal/messaging/messaging_pyx.so
    /OP091/common/params_pyx.so
  --- do NOT use "move", use "copy" !!!
  --- see Ch3. and add "json11"
  (DP091venv) jinn@Liu:~/openpilot$ scons -u -j$(nproc)
    scons: done building targets.
  --- OK.

  Ch8. do 'rednose/SConscript' Line 398
  --- add
    /openpilot/rednose/SConscript
    `rednose/helpers/common_ekf.cc' not found
    `rednose/helpers/ekf_sym.cc' not found
  --- OK. done

  Ch9. do 'system/camerad, /clocksd, /proclogd/SConscript' Line 401
  --- add
    /home/jinn/openpilot/system/camerad/SConscript
    /home/jinn/openpilot/system/camerad/cameras/camera_common.cc
    /home/jinn/openpilot/system/camerad/cameras/camera_qcom2.cc
    /home/jinn/openpilot/system/camerad/cameras/camera_util.cc
    /home/jinn/openpilot/system/clocksd/SConscript
    /home/jinn/openpilot/system/clocksd/clocksd.cc
    /home/jinn/openpilot/system/proclogd/SConscript
    /home/jinn/openpilot/system/proclogd/main.cc
    /home/jinn/openpilot/system/proclogd/proclog.cc
  --- Error
    ./system/camerad/cameras/camera_util.h:8:10: fatal error: 'media/cam_req_mgr.h' file not found
    #include <media/cam_req_mgr.h>
      /home/jinn/OP091/third_party/linux/include/media/cam_req_mgr.h
      /home/jinn/openpilot/system/camerad/include/media/cam_req_mgr.h
  --- move /media to
      /home/jinn/openpilot/third_party/linux/include/media
  --- OK move
  --- Error
    /usr/bin/ld: cannot find -lyuv
  --- Change Line 21 in system/hardware/hw.h
    return Hardware::PC() ? util::getenv("HOME") + "/.comma/media/0/realdata" : "/data/media/0/realdata";
  --- add
    /home/jinn/openpilot/third_party/libyuv/x64
  --- OK. done

  Ch10. do 'system/logcatd/SConscript' Line 407
  --- add
    /home/jinn/openpilot/system/logcatd/SConscript
    /home/jinn/openpilot/system/logcatd/logcatd_systemd.cc
  --- OK. done

  Ch11. do 'body/board/SConscript', 'cereal/SConscript',
           'opendbc/can/SConscript', 'panda/SConscript' Line 412
  --- add
    /home/jinn/openpilot/body/SConstruct
    /home/jinn/openpilot/body/board/SConscript
    /home/jinn/openpilot/cereal/SConstruct
    /home/jinn/openpilot/opendbc/can/common.cc
    /home/jinn/openpilot/opendbc/can/dbc.cc
    /home/jinn/openpilot/opendbc/can/packer.cc
    /home/jinn/openpilot/opendbc/can/packer_pyx.cpp
    /home/jinn/openpilot/opendbc/can/packer_pyx.pyx
    /home/jinn/openpilot/opendbc/can/parser.cc
    /home/jinn/openpilot/opendbc/can/parser.cc
    /home/jinn/openpilot/opendbc/can/parser_pyx.pyx
  --- OK. done

  Ch12. do 'third_party/SConscript' Line 419
  --- OK. done

  Ch13. do 'common/kalman/SConscript' Line 421
  --- add
    /home/jinn/openpilot/common/kalman/SConscript
  --- OK. done

  Ch14. do 'common/transformations/SConscript' Line 422
  --- add
    /home/jinn/openpilot/common/transformations/SConscript
    /home/jinn/openpilot/common/transformations/coordinates.cc
    /home/jinn/openpilot/common/transformations/orientation.cc
  --- OK. done

  Ch15. do 'selfdrive/modeld/SConscript' Line 424
  --- add

230324: OK. PC: Change scons "DP091L", get "DP091a"
--- Change DP091 from C2 to DP091L == PC/openpilot. See 230319 Read ToDo.

  Ch1. Change PC/openpilot to PC/OP091 and PC/DP091 to PC/openpilot == PC/DP091L (L: long, S: short)
    jinn@Liu:~$ gedit ~/.bashrc
      -- rc: "run commands"
      #source /home/jinn/openpilot/tools/openpilot_env.sh
      export PATH=$PATH:$HOME/.local/bin
      #export PYENV_ROOT="$HOME/.pyenv"
      #export PATH="$PYENV_ROOT/bin:$PATH"
      #eval "$(pyenv init --path)"
      #eval "$(pyenv virtualenv-init -)"
      # . ~/.pyenvrc

  Ch2. Do "venv", "ubuntu_setup.sh" in Step (B) in InstallOP.docx for DP091L
    -- add these to /openpilot/..
      /OP091/tools/openpilot_env.sh
      /OP091/tools/ubuntu_setup.sh
      /OP091/pyproject.toml
      /OP091/poetry.lock
      /OP091/update_requirements.sh
      /OP091/.python-version
    -- do venv
      jinn@Liu:~/openpilot$ git submodule update --init
      jinn@Liu:~$ python -m venv ~/DP091venv
      jinn@Liu:~$ . ~/DP091venv/bin/activate
      (DP091venv) jinn@Liu:~$ pyenv versions
        * 3.8.2 (set by /home/jinn/.pyenv/version)
      (DP091venv) jinn@Liu:~/openpilot$ poetry shell
      (DP091venv) jinn@Liu:~/openpilot$ tools/ubuntu_setup.sh
        WARNING: Can not proceed with installation. Kindly remove the
          '/home/jinn/.pyenv' directory first.
      -- rename .pyenv to .pyenv_OP091
      -- add /home/jinn/.pyenv/versions/3.8.2
      -- close terminal
      -- redo
      jinn@Liu:~/openpilot$ git submodule update --init
      jinn@Liu:~$ python -m venv ~/DP091venv
      jinn@Liu:~$ . ~/DP091venv/bin/activate
      (DP091venv) jinn@Liu:~/openpilot$ pyenv versions
        system
        3.8.2
        3.8.2/envs/YPN
      * 3.8.10 (set by /home/jinn/openpilot/.python-version)
      (DP091venv) jinn@Liu:~/openpilot$ poetry shell
      (DP091venv) jinn@Liu:~/openpilot$ tools/ubuntu_setup.sh
      ----   OPENPILOT SETUP DONE   ----
        Open a new shell or configure your active shell env by running:
        source ~/.bashrc
      (DP091venv) jinn@Liu:~/openpilot$ source ~/.bashrc

  Ch3. Do 'common/SConscript' Line 349 in /openpilot/SConstruct
    -- added these files to /openpilot
    /openpilot/SConstruct
    /openpilot/common/SConscript

    jinn@Liu:~/openpilot$ . ~/DP091venv/bin/activate
    (DP091venv) jinn@Liu:~/openpilot$ scons --version
      SCons by Steven Knight et al.:
        SCons: v4.4.0
    -- if not v4.4.0
    (DP091venv) jinn@Liu:~$ python -m pip install scons==4.4.0
    (DP091venv) jinn@Liu:~$ deactivate
    jinn@Liu:~/openpilot$ . ~/DP091venv/bin/activate
    (DP091venv) jinn@Liu:~/openpilot$ scons -u -j$(nproc)
    -- Error
      scons: *** [common/clutil.o] Source `common/clutil.cc' not found, needed by target `common/clutil.o'.
      scons: *** [common/gpio.o] Source `common/gpio.cc' not found, needed by target `common/gpio.o'.
      ...
    -- add all above files
    (DP091venv) jinn@Liu:~/openpilot$ scons -u -j$(nproc)
    -- Error
      /usr/bin/ld: cannot find -ljson11
      scons: *** [common/params_pyx.so] Error 1
    -- check and add all missing files for Lines 60, 73, 76, 78, 87, 139-144,
         193-210, 222-226, 314, 319, 332 in /home/jinn/openpilot/SConstruct
    -- find "json11" in /openpilot/SConstruct and then add files
      /openpilot/third_party/libjson11.a
      /openpilot/third_party/json11/json11.o
    -- output
      scons: done building targets.
    -- OK.

  Ch4. Change /openpilot to /DP091L and /OP091 to /openpilot
    jinn@Liu:~$ gedit ~/.bashrc
    -- no change
    -- rename .pyenv to .pyenv_DP091L and .pyenv_OP091 to .pyenv
    -- mv /.pyenv_DP091L/versions/3.8.2 to /.pyenv/versions/3.8.2
    jinn@Liu:~/openpilot$ . ~/sconsvenv/bin/activate
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
    (sconsvenv) jinn@Liu:~/openpilot/tools/replay$
      ./replayJLL --data_dir dataC "8bfda98c9c9e4291|2020-05-11--03-00-57--61"
    -- OK.

  Ch5. Change all /DP091L back to /openpilot. Do SConscript(['cereal/SConscript'])
  (DP091venv) jinn@Liu:~/openpilot$ scons -u -j$(nproc)
  -- OK.
  -- find "cereal" in /openpilot/SConstruct and then add files
    *** [cereal/messaging/messaging.os]
    'cereal/messaging/messaging.cc' not found
    fatal error: 'dp.capnp.h' file not found
    *** [cereal/gen/cpp/log.capnp.os] Error 1
    `cereal/messaging/impl_zmq.cc' not found
    *** [cereal/messaging/impl_msgq.os]
  -- get 'dp.capnp.h' from https://github.com/dragonpilot-community/dragonpilot
  -- !!! stop scons DP091L. use DP091a downloaded from this github on 230327.
--- get "DP091a" from https://github.com/dragonpilot-community/dragonpilot

--- Difference:
  /OP091 = /PC/openpilot, /DP091 = /C2/data/openpilot = /PC/DP091

  /OP091/selfdrive/loggerd/loggerd
  /OP091/selfdrive/loggerd/encoderd
  /OP091/selfdrive/loggerd/bootlog
  /OP091/selfdrive/loggerd/SConscript
   DP091/selfdrive/loggerd/ "none of these"

  /OP091/selfdrive/ui/SConscript091
   DP091/selfdrive/ui/ "no SConscript"
  /OP091/selfdrive/ui/installer/
   DP091/selfdrive/ui "no /installer"
   DP091/installer
   DP091/installer/media/0/dashcam/2023-03-18_02-03-21.mp4
  /OP091/selfdrive/ui/qt/widgets/offroad_alerts.cc
   DP091/selfdrive/ui/qt/ "no widgets folder"

  /data/media/0/dashcam/ "empty"
  /data/media/0/realdata/
  /data/media/0/fakedata/
