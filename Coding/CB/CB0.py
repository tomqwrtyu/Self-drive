CB 0  Jinn-Liang Liu  2020.4.28 - 2023.1.17  All Rights Reserved. © Copyright 2020-2023

"openpilot (OP)", "dragonpilot (DP)", "Comma Two (C2A/C2B)", and "Comma Body (CB)" Notes

========== Old C2B5Notes ==========
C2 (comma two) B5 (Model B5)
--- rename C2oldNotes.docx to CB0.py

===== Road Tests

220304: OP0.8.13 shows “red” pathway and does not show lane lines. Change back to 0.8.12.
211125: Yes! modelB3.dlc replaces supercombo079 and can run on C2 but drives like a drunker.

===== Stop and Go Begin

230110: OK: Stop and Go (OP0812 + smartDSU in Toyota Prius)
  Saber (from DP): change /data/openpilot/selfdrive/car/toyota/carcontroller.py
  Lines 64, 65 Old:
    if CS.out.standstill and not self.last_standstill and CS.CP.carFingerprint not in NO_STOP_TIMER_CAR:
      self.standstill_req = True
  Lines 64, 65 New:
    #if CS.out.standstill and not self.last_standstill and CS.CP.carFingerprint not in NO_STOP_TIMER_CAR:
    #  self.standstill_req = True
  Result: OK. Great! My year-long SnG problem has been solved by this change. Saber, thank you.

230108: Stop and Go (OP0812 + smartDSU in Toyota Prius)
  Wang: change /selfdrive/controls/lib/pathplanner.py
  Line 216 Old:
    plan_send.pathPlan.mpcSolutionValid = bool(plan_solution_valid)
  Line 216 New:
    plan_send.pathPlan.mpcSolutionValid = Ture
  Note: The file pathplanner.py was renamed to the following in OP0812:
    selfdrive/controls/lib/lateral_planner.py which is (mpc is) for “lateral” control
    but SnG is a “longitudinal” control. So I think this change may not work.
    I’ve not yet road tested this change.

230108: Stop and Go (OP0812 + smartDSU in Toyota Prius)
  Saber (from DP): change
  /data/openpilot/selfdrive/car/toyota/carcontroller.py
  Lines 64, 65 Old:
    if CS.out.standstill and not self.last_standstill and CS.CP.carFingerprint not in NO_STOP_TIMER_CAR:
      self.standstill_req = True
  Lines 64, 65 New:
    #if CS.out.standstill and not self.last_standstill and CS.CP.carFingerprint not in NO_STOP_TIMER_CAR:
    self.standstill_req = True
  Result: NG. I need to step on the gas pedal (thus disengaged OP) to drive the car
    after OP stopped at 0 speed.

230103: Stop and Go (OP0812 + smartDSU in Toyota Prius)
  change /data/openpilot/selfdrive/car/toyota/carcontroller.py
  using carcontroller090.py (Line 85)
  Line 64 Old:
    if CS.out.standstill and not self.last_standstill and CS.CP.carFingerprint not in NO_STOP_TIMER_CAR:
  Line 64 New:
    if CS.out.standstill and not self.last_standstill and (self.CP.carFingerprint not in NO_STOP_TIMER_CAR or self.CP.enableGasInterceptor):
  Result: NG. Error on C2: TAKE CONTROL IMMEDIATELY Controls Unresponsive
    Does enableGasInterceptor mean that Prius needs a comma pedal to achieve automatic SnG?
    Prius is a NO_STOP_TIMER_CAR.
    # no resume button press required
    NO_STOP_TIMER_CAR = TSS2_CAR | set([CAR.RAV4H, CAR.HIGHLANDERH, CAR.SIENNA, CAR.LEXUS_ESH])
===== Stop and Go End

===== Comma body Begin

230118: OK Install: DP091 = DPbeta2 on C2+body/car
  C2+PC => C2 Fastboot Mode => jinn@Liu:~/eon-neos$ ./flash.sh  => Continue to Setup
  => Connect to WiFi => Custom Software => DPbeta2 => Install Software => Begin Training
  => ADD jinnliu => FTP DPbeta2 (0.9.1) from C2 to PC (backup copy) => DP - Cars: COMMA BODY
  => Error: “openpilot Unavailable Waiting for controls to start” (应该是panda没连接好)
  Change DPbeta2/selfdrive/manager/process_config.py
  Line 65 Old:
    # PythonProcess("webjoystick", "tools.joystick.web", onroad=False, callback=notcar),
  Line 65 New:
    PythonProcess("webjoystick", "tools.joystick.web", onroad=False, callback=notcar),
  => Error: same => Joystick
  => root@localhost:/data/openpilot/tools/joystick$ python web.py => http://10.0.0.6:5000/
  => Error: same => Panda was NOT properly connected. Adjusted it.
  OK: body is walking! It keeps slowly turning around at the same spot without moving
  ahead or backward.
  => root@localhost:/data/openpilot/tools/joystick$ python web.py
   * Serving Flask app 'web' (lazy loading)
  …
    File "web.py", line 78, in <module>
      main()
    File "web.py", line 75, in main
      app.run(host="0.0.0.0")
    File "/data/data/com.termux/files/usr/lib/python3.8/site-packages/flask/app.py", line 920, in run
      run_simple(t.cast(str, host), port, self, **options)
    File "/data/data/com.termux/files/usr/lib/python3.8/site-packages/werkzeug/serving.py", line 1010, in run_simple
      inner()
    File "/data/data/com.termux/files/usr/lib/python3.8/site-packages/werkzeug/serving.py", line 950, in inner
      srv = make_server(
    File "/data/data/com.termux/files/usr/lib/python3.8/site-packages/werkzeug/serving.py", line 782, in make_server
      return ThreadedWSGIServer(
    File "/data/data/com.termux/files/usr/lib/python3.8/site-packages/werkzeug/serving.py", line 688, in __init__
      super().__init__(server_address, handler)  # type: ignore
    File "/data/data/com.termux/files/usr/lib/python3.8/socketserver.py", line 452, in __init__
      self.server_bind()
    File "/data/data/com.termux/files/usr/lib/python3.8/http/server.py", line 138, in server_bind
      socketserver.TCPServer.server_bind(self)
    File "/data/data/com.termux/files/usr/lib/python3.8/socketserver.py", line 466, in server_bind
      self.socket.bind(self.server_address)
  OSError: [Errno 98] Address already in use
  => http://10.0.0.6:5000/ => PC browser shows the “ball” joystick but it cannot control the body.
  => Need QR code to pair C2 with iPhone?
  => C2: Joystick Mode, Gas: 0%, Steer: 0%, 0.0 REAL STEER, 44.0  RAM PER, OFF ENG RPM
  => C2: Reboot  =>  This site can’t be reached 10.0.0.6 refused to connect.
  => Body Problem: sudden large movement when the power button is slightly pressed again (Engine On).
  => C2 body face again (not in Joystick Mode )
  =>  http://10.0.0.6:5000/
  => OK: I can control body from PC!
  Conclusion: So far, body can only balance and roll.
    It cannot find the pathway to go, does not break, and always turns to the
    right in turning. It seems to calibrate (from turning around to fast roll
    to slow roll to normal roll) pretty long.

230116: Use TWRP to Install DP0.8.17 on C2+body
  C2+USB+PC =>  Flash TWRP twrp.img to C2 => “Fastboot Mode”
  => jinn@Liu:~/twrp$ fastboot devices => $ fastboot oem unlock
  => $ fastboot flash recovery twrp.img => $ fastboot boot twrp.img
  => exit TWRP => Reboot => System
  => FTP /home/jinn/twrp/DPimg to C2/data/media/0/TWRP/77c2722e/DPimg (使用 TWRP 恢复 openpilot 备份)
  => Error: TWRP can see the folder DPimg but cannot see its contents (maybe due to
    the zip file was created for Windows not Ubuntu). So, I unzip it in Windows to
    /DP220922
  => OK => Reboot System => OK

230115: OK Install: Dragonpilot DPbeta2 = DP091 on C2+body/car
--- DPbeta2 from https://smiskol.com/fork/dp/beta2
  C2+PC => C2 Fastboot Mode => jinn@Liu:~/eon-neos$ ./flash.sh
  => Continue to Setup => Continue to WiFi => Custom Software
  => https://smiskol.com/fork/dp/0.8.9 => Language
  => 只要找底下有三組XXX(XX)，點選中間那個就是中文(繁體) => 返回重開機
  => OK
  Do Step 1 with https://smiskol.com/fork/dp/beta2
  => … dragon 2022.12.30/beta2 => Device  => Change Language => 中文(繁體)
  => OK: Car.

  Wrong Step: turn on Portable Wi-Fi hotspot weedle-77c2722e
  => Portable Wi-Fi hotspot setup: Password: jxxxxxx2023
  => iPhone: Wi-Fi connected to weedle-77c2722e (OK)

  Failed to install OP0.8.16 due to insufficient memory?
  No. C2 cannot install 0816, so change to DragonPilot.

  root@localhost:/data$ rm -rf openpilot && git clone -b v0.8.16 https://github.com/commaai/openpilot && scons -i && reboot
  …
  scons: *** No SConstruct file found.
  cd /data && rm -rf openpilot && git clone https://gitee.com/afaaa/kegman openpilot && cd openpilot && git checkout kegman-0.7.3 && scons -i && reboot

230114: Conclusion of trying to Install OP0.8.16 on C2 with OP0.8.12
  Use Method 1 to install. Failed. It installed OP0.8.13.1.
  Use Method 2 to install. Failed. It installed OP0.8.12.
  Use Method 3 to install. Failed. It installed OP0.8.13.1 => OP0.8.16 => NG
  Questions: How to put OP0.8.16 in these methods (in which Step)?
  Where and what is TWRP 备份 in Method 2?

230114: Use commaai/eon-neos to Install OP0.8.16 on C2 from PC with Ubuntu 20.04
  Connect C2 to PC by a USB cable
  jinn@Liu:~$ git clone https://github.com/commaai/eon-neos.git
  jinn@Liu:~/eon-neos$ ./download.py
  .....hash check pass
  C2 Fastboot Mode => jinn@Liu:~/eon-neos$ ./flash.sh => Continue to Setup
  => Continue to WiFi => Custom Software =>
  https://github.com/commaai/openpilot/tree/v0.8.16 (NG)
  https://openpilot.comma.ai => OP0813.1 (NG)
  root@localhost:/data$ reboot
--- Error: C2 screen nothing. There is no openpilot/installer folder from 0.8.13

230113: Use TWRP to Install OP0.8.16 on C2 from PC with Ubuntu 20.04
  Check C2 Serial #:
  C2 Power On => OP0812 => touch “Device” on C2 => Serial 77c2722e
  Install ADB & Fastboot
  jinn@Liu:~$ sudo apt update
  jinn@Liu:~$ sudo apt install android-tools-adb android-tools-fastboot
  jinn@Liu:~$ adb version
  Android Debug Bridge version 1.0.39
  …
  Installed as /usr/lib/android-sdk/platform-tools/adb
  Do Fastboot Mode on C2:
  C2 Power Off => connect C2 to PC by a USB cable
  => hold “volume down” key and “power” together until C2 screen shows “Fastboot Mode”
  => on PC:
  jinn@Liu:~$ fastboot devices
  77c2722e	fastboot
  Download TWRP recovery.img and rename it to twrp.img then flash it to C2.
  jinn@Liu:~/twrp$ fastboot devices
  77c2722e	fastboot
  jinn@Liu:~/twrp$ ls
  twrp.img
  jinn@Liu:~/twrp$ fastboot oem unlock
  ...
  (bootloader) 	Device already : unlocked!
  jinn@Liu:~/twrp$ fastboot flash recovery twrp.img
  …
  OKAY [  0.856s]
  jinn@Liu:~/twrp$ fastboot boot twrp.img
  downloading 'boot.img'...
  OKAY [  2.437s]
  booting...
  OKAY [  0.357s]
--- C2 screen shows TWRP menu =>
  Swipe to confirm Flash => touch “Backup” =>
  Backup Folder: /data/media/0/TWRP/BACKUPS/77c2722e/2023-....6.0.1
  … done
  => touch “Reboot System” => Back to OP0812 (NG)

230111: TWRP, Fastboot
  Saber: 有幾個相關的網站
  1. 進fastboot刷recovery.img Windows 下手动分布安装 openpilot 教程
  2. Twrp還原是這樣 使用 TWRP 恢复 openpilot 备份

  bricked phone > unbrick your phone (Steps 1, 2, 3, 4) > TWRP > Fastboot
  Fastboot: C2 Fastboot (reflash) Recovery mode (for Factory Reset),
  TWRP: https://drive.google.com/drive/folders/1WaebI2JGgISa3WM15zIP0lthFTjJcXVt?usp=sharing

  All Android devices ship with a recovery environment preinstalled.
  This recovery software can be used to restore the device to factory default settings,
    update its operating system, and perform other diagnostic tasks.
  TWRP is the most popular custom recovery for Android, which is a replacement for the
    stock recovery mode. It's used to root your phone, flash custom ROMs, unbrick your device,
    and much more.
  A custom ROM replaces your device’s Android operating system — normally stored in
    read-only memory — with a new version of the Android operating system.
  Team Win Recovery Project (TWRP) is an open-source software custom recovery image for
    Android-based devices. It provides a touchscreen-enabled interface that allows users
    to install third-party firmware and back up the current system which are functions
    often unsupported by stock recovery images.
  The Android recovery mode is a mode of Android used for installing updates.
  Recovery Image means a copy of the Image originally installed on an Embedded System,
    contained on separate media.
  Android recovery image is a bootable program on an Android flash memory partition that
    is used to perform a factory reset or restore the original OS version.
  A flash or flashing is a term that describes updating the code (firmware) on a chip.

  Fastboot allows us to boot a device from a custom recovery image.
    It is most commonly used to install a custom recovery.
    fastboot flash recovery [filename.img] Installs a custom recovery, such as TWRP, on your device.
    For ease of use, we suggest changing the recovery filename to
    something easy—twrp.img, for instance—and moving it into the platform-tools folder.
  Fastboot is a protocol included with the Android SDK package used primarily to modify
    the flash filesystem via a USB connection from a host computer.
  On Android, Fastboot is a diagnostic tool. It's essential if you need to unbrick your phone.
  A flash file system is a file system designed for storing files on
    flash memory–based storage devices.
  Flash memory is an electronic non-volatile computer memory storage medium
    that can be electrically erased and reprogrammed.

  How to Unbrick Your Android Phone
  What is a Custom Recovery on Android
  TWRP: A Complete Guide to the Custom Android Recovery
  The Complete Guide to Rooting Your Android Phone or Tablet
  How to Use ADB and Fastboot on Android

230111: I failed to Install OP0.8.14 or 0.8.16 on C2 but did Install 0.8.12 OK.
  Saber, can you help? Thanks.

230111: Install OP 0814 for C2+body
  On C2, Software => click UNINSTALL => Continue to Setup => Wi-Fi => password
  => go to OP on C2 =>  Entering Recovery Mode 1 => update required
  => Entering Recovery Mode 2 =>  v0.8.13.1-release  =>
  jinn@Liu:~/.ssh$ ssh -T git@github.com
--- Generating a new GitHub SSH key
  jinn@Liu:~$ ssh-keygen -t ed25519 -C "jinnliangliu@gmail.com"
  Generating public/private ed25519 key pair.
  Enter file in which to save the key (/home/jinn/.ssh/id_ed25519):
  Enter passphrase (empty for no passphrase):
  Enter same passphrase again:
--- I got two keys in /home/jinn/.ssh/id_ed25519 (private key) and id_ed25519.pub (public key)
--- Adding id_ed25519.pub to the ssh-agent
  jinn@Liu:~$ eval "$(ssh-agent -s)"
  Agent pid 155759
  jinn@Liu:~$ ssh-add ~/.ssh/id_ed25519
  Identity added: /home/jinn/.ssh/id_ed25519 (jinnliangliu@gmail.com)
  Enter passphrase for /home/jinn/.ssh/id_ed25519:
  Identity added: /home/jinn/.ssh/id_ed25519 (jinnliangliu@gmail.com)
--- Add id_ed25519.pub to my GitHub
--- Before adding a new SSH key to your account on GitHub.com, you should have
      Checked for existing SSH keys
  jinn@Liu:~$ ls -al ~/.ssh
    . Generating a new SSH key and adding it to the ssh-agent
  jinn@Liu:~$ ssh-keygen -t ed25519 -C "jinnliangliu@gmail.com"
--- check the key on GitHub
  jinn@Liu:~/.ssh$ ssh -T git@github.com
  git@github.com: Permission denied (publickey).
--- Important: Do following if not OK.
  jinn@Liu:~/.ssh$ gedit id_ed25519.pub
--- Copy the contents of id_ed25519.pub (probably starts with ssh-ed25519, this also
  in your public key file) and add it to https://github.com/settings/ssh/new
--- my GitHub Name: JL => Title: C2 => In Key: paste the copied contents
--- then click “add SSH key”
  jinn@Liu:~/.ssh$ cp known_hosts known_hosts_copy
--- delete all in known_hosts to empty
  jinn@Liu:~/.ssh$ gedit known_hosts
  jinn@Liu:~/.ssh$ ssh -T git@github.com
  Warning: Permanently added the ECDSA host key for IP address '13.114.40.48' to the list of known hosts.
  Hi jinnliu! You've successfully authenticated, but GitHub does not provide shell access.
  jinn@Liu:~/.ssh$ gedit known_hosts
--- three new lines in known_hosts
--- Remove GitHub username
  C2 => Network => jinnliu (click REMOVE) => Add jinnliu again => on C2 screen
  => get SSH Keys jinnliu
  jinn@Liu:~$ ssh root@10.0.0.16
  root@localhost:/data/openpilot$
--- OK
--- Install 0.8.16 on NEOS 20
  root@localhost:/data$ rm -rf openpilot && git clone -b v0.8.16 https://github.com/commaai/openpilot
  root@localhost:/data$ reboot
--- Error: C2 screen nothing. There is no openpilot/installer folder from 0.8.13 on.

--- Install 0.8.16
  root@localhost:/data$ rm -rf openpilot && git clone -b v0.8.16 https://github.com/commaai/openpilot
  root@localhost:/data$ reboot
--- Error: C2 screen nothing. There is no openpilot/installer folder from 0.8.13 on.
--- FTP /home/jinn/OP0812/installer to root@localhost:/data/openpilot
  root@localhost:/data/openpilot/installer/updater$ wget https://cdn.discordapp.com/attachments/538741329799413760/774123747257876480/update.json && reboot
--- Error: Same C2 screen nothing.

230110: OP0.8.14 supports comma body
--- Install 0.8.16
  jinn@Liu:~$ ssh root@10.0.0.6
  root@localhost:/data$ rm -rf openpilot && git clone -b v0.8.16 https://github.com/commaai/openpilot
  root@localhost:/data$ reboot
--- Error: C2 screen nothing. There is no openpilot/installer folder from 0.8.13 on.
--- FTP /home/jinn/OP0812/installer to root@localhost:/data/openpilot
  root@localhost:/data/openpilot/installer/updater$ wget https://cdn.discordapp.com/attachments/538741329799413760/774123747257876480/update.json && reboot
--- Error: Same C2 screen nothing.
--- Install 0.8.12
  root@localhost:/data$ rm -rf openpilot && git clone -b v0.8.12 https://github.com/commaai/openpilot && cd openpilot/installer/updater
  root@localhost:/data/openpilot/installer/updater$ wget https://cdn.discordapp.com/attachments/538741329799413760/774123747257876480/update.json && reboot
--- Install 0.8.16 from C2 screen
  Software => Check for Update => tap “Check”

===== Comma body End

Saber: 這次提到的開源範例 Openpilot 就是 MIT 授權，MIT 授權的衍伸品是可以商業化的，
  但是你的產品與銷售必須要放置原作與作者的版權說明，或是標示原作者的出處，才能被視為是經作者許可的授權。
  如果違反這點，衍伸品的 MIT 授權就是無效的。

========== Old C2Notes ==========

230110: install OP0.8.16 to run body
--- install 0.8.16
  jinn@Liu:~$ ssh root@10.0.0.6
  root@localhost:/data$ rm -rf openpilot && git clone -b v0.8.16 https://github.com/commaai/openpilot
  root@localhost:/data$ reboot
--- Error: C2 screen nothing. There is no openpilot/installer folder from 0.8.13 on.

221208: re-install OP0.8.12 as C2 failed to operate in car
jinn@Liu:~$ ssh root@10.0.0.16
root@localhost:/data$ rm -rf openpilot && git clone -b v0.8.12 https://github.com/commaai/openpilot && cd openpilot/installer/updater
root@localhost:/data/openpilot/installer/updater$ wget https://cdn.discordapp.com/attachments/538741329799413760/774123747257876480/update.json && reboot
root@localhost:/data/openpilot$ echo -en "1" > /data/params/d/DisableUpdates
--- car: self-driving OK but still with WARNING: This branch is not tested
--- download update.json
jinn@Liu:~/OP$ wget https://cdn.discordapp.com/attachments/538741329799413760/774123747257876480/update.json

220304: install OP0.8.12
--- C2: uninstall and install 0.8.13
--- C2: => Network => ADD => jinnliu
root@localhost:/data$ rm -rf openpilot && git clone -b v0.8.12 https://github.com/commaai/openpilot && cd openpilot/installer/updater && rm update.json && wget https://cdn.discordapp.com/attachments/538741329799413760/774123747257876480/update.json && reboot
root@localhost:/data/openpilot$ echo -en "1" > /data/params/d/DisableUpdates
--- car: self-driving OK but still with WARNING: This branch is not tested

freeze OP updates:
/selfdrive/manager.py  >>  # "updated": "selfdrive.updated"
(this line does not exist in v0.8.12).
Or enter this command:  echo -en "1" > /data/params/d/DisableUpdates

220105:
  root@localhost:/data$  echo -en "1" > /data/params/d/DisableUpdates
  root@localhost:/data$  vim /data/params/d/DisableUpdates

211221: ssh to C2 and run ypreg.dlc by uiview.py on C2
--- SSH to comma two
--- do Option 2 (not 1) on Ubuntu:
--- openpilot 0.8.3 mandates usage of keys from a personal GitHub account and
      changes the SSH port from 8022 to 22.
--- Generating a new GitHub SSH key
  jinn@Liu:~$ ssh-keygen -t ed25519 -C "jinnliangliu@gmail.com"
    Generating public/private ed25519 key pair.
    Enter file in which to save the key (/home/jinn/.ssh/id_ed25519):
    Enter passphrase (empty for no passphrase):
    Enter same passphrase again:
--- I got two keys in /home/jinn/.ssh/id_ed25519 (private key) and id_ed25519.pub (public key)
--- Adding id_ed25519.pub to the ssh-agent
  jinn@Liu:~$ eval "$(ssh-agent -s)"
    Agent pid 56012
  jinn@Liu:~$ ssh-add ~/.ssh/id_ed25519
    Enter passphrase for /home/jinn/.ssh/id_ed25519:
    Identity added: /home/jinn/.ssh/id_ed25519 (jinnliangliu@gmail.com)
--- Add id_ed25519.pub to my GitHub
--- Before adding a new SSH key to your account on GitHub.com, you should have:
      . Checked for existing SSH keys
  jinn@Liu:~$ ls -al ~/.ssh
    . Generating a new SSH key and adding it to the ssh-agent
  jinn@Liu:~$ ssh-keygen -t ed25519 -C "jinnliangliu@gmail.com"
--- check the key on GitHub
  jinn@Liu:~/.ssh$ ssh -T git@github.com
--- The authenticity of host 'github.com (13.114.40.48)' can't be established.
    ECDSA key fingerprint is SHA256:p2QAMXNIC1TJYWeIOttrVc98/R1BUFWu3/LiyKgUfQM.
    Are you sure you want to continue connecting (yes/no/[fingerprint])? yes
    Warning: Permanently added 'github.com,13.114.40.48' (ECDSA) to the list of known hosts.
    git@github.com: Permission denied (publickey).
--- Important: Do following if not OK.
--- Copy the contents of id_ed25519.pub (probably starts with ssh-ed25519, this also in
    your public key file) and add it to https://github.com/settings/ssh/new
--- then click “add SSH key”
  jinn@Liu:~/.ssh$ cp known_hosts known_hosts_copy
--- delete all in known_hosts to empty
  jinn@Liu:~/.ssh$ gedit known_hosts
  jinn@Liu:~/.ssh$ ssh -T git@github.com
    Warning: Permanently added the ECDSA host key for IP address '13.114.40.48' to the list of known hosts.
    Hi jinnliu! You've successfully authenticated, but GitHub does not provide shell access.
  jinn@Liu:~/.ssh$ gedit known_hosts
--- three new lines in known_hosts
--- Remove GitHub username C2 => Network => jinnliu (click REMOVE)
    => Add jinnliu again => on C2 screen => get SSH Keys   jinnliu
  jinn@Liu:~$ ssh root@10.0.0.16
    root@localhost:/data/openpilot$
--- OK
--- OP079-081
--- C2 => Authorized SSH Keys => Edit => GitHub Username jinnliu
  jinn@Liu:~$ chmod 600 c2key.pem
  jinn@Liu:~$ ssh root@10.0.0.12 -p 8022 -i c2key.pem
    root@localhost:/system/comma/home$
--- OK
--- Install FileZilla (FZ on Ubuntu or Windows) with c2key.ppk to transfer files (FTP)
    back and forth (easier for C2 coding).
--- Run ypreg.dlc
  1. jinn@Liu:~$ ssh root@10.0.0.12
--- we are now in c2
--- to exit c2
  root@localhost:/data/openpilot$ logout
  Connection to 10.0.0.12 closed.
  root@localhost:/data/openpilot$ cd ..
  root@localhost:/data$ mv openpilot openpilot089
  root@localhost:/data$ mv openpilot079 openpilot
  root@localhost:/data$ reboot
--- C2 said “An update to NEOS is required.”
--- C2 said “There was an error” after reboot.
--- Conclusion: Can’t use old version OP079.
  root@localhost:/data$ mv openpilot openpilot079
  root@localhost:/data$ mv openpilot089 openpilot
  root@localhost:/data$ reboot
--- use OP089 not 079
  4. FTP ypreg.dlc and supercombo.dlc to root@10.0.0.12:/data/openpilot/models
  root@localhost:/data/openpilot/models$ ls
    dmonitoring_model_q.dlc  supercombo.thneed
    supercombo.dlc		 ypreg.dlc
  5. rebuild OP (it takes more than 30 mins)
  root@localhost:/data/openpilot$ scons
    ...
    scons: done building targets.
  6. tmux on c2 (Terminal Multiplexing)
  root@localhost:/data/openpilot$ tmux at
--- now in bash 0
  7. kill all *d: type ctrl + c to kill all background demon processes (*d like thermald)
  everything is dead
  8. type ` then c (` c) and now in bash 1
  9. run ypreg.dlc by uiview079yp.py
  root@localhost:/data/openpilot/selfdrive/debug$ mv uiview.py uiview089.py
  root@localhost:/data/openpilot/selfdrive/debug$ mv uiview079yp.py uiview.py
  root@localhost:/data/openpilot/selfdrive/debug$ python uiview.py
--- Done. See output on C2 screen.
  10. type ctrl + c to kill uiview
  11. exit tmux (both bash 0 and 1)
  $ tmux kill-session -t comma
  root@localhost:/system/comma/home$ reboot
--- OK. Done. See screenshots for more.
  Get Toyota Fingerprint (C2 on car): see zcCS5.py

211125: downgrade to OP0.7.9
--- C2 failed to reboot OP079C2a and OP079C2 by FZ => openpilot
--- Install openpilot 0.7.9
  root@localhost:/data$ rm -rf openpilot
  root@localhost:/data$ git clone -b v0.7.9 https://github.com/commaai/openpilot.git
--- reboot
--- Error on C2 screen
    There was an error - failed to download recovery
--- get update.json of old NEOS, save update079.json, update_old.json
  root@localhost:/data/openpilot$ cd installer/updater
  root@localhost:/data/openpilot/installer/updater$ mv update.json update079.json
  root@localhost:/data/openpilot/installer/updater$
    wget https://cdn.discordapp.com/attachments/538741329799413760/774123747257876480/update.json
--- save this key file to /home/jinn/OP/update_old.json
  root@localhost:/data/openpilot/installer/updater$ ls
  root@localhost:/data/openpilot/installer/updater$ reboot
--- OP079 OK.

201023YP: 目前model執行速度還有影像載入速度都已經正常.
--- Part 1: C2 Simulator
  Plans:
  1.根據我們的Steering angle 我可以像以前一樣在Comma2上畫出線 跟預測道路
201021YP: htop (memory), python uiview.py (proc ... modeld) tmax at, slow frame(680), time: 91.96
  2.將steering angle 的角度加到controlers 上讓車子去實際跑
  3.將物體檢測模型加進去
  #define LONG_X_IDX LEAD_IDX + MDN_GROUP_SIZE*LEAD_MDN_N + SELECTION
  #define LONG_V_IDX LONG_X_IDX + TIME_DISTANCE*2

201017YP:  trafficd.cc (yuv to rgb; dlc:
  input_dim=[['input_imgs', '1,12,128,256']; 12/2 = 6, y:4, u:1, v:1;

--- Part 2: Controls

  Interface.py
  selfdrive/car/toyota/interface.py,  carcontroller.py, toyotacan.py
  selfdrive/controls/controlsd.py, plannerd.py
  selfdrive/controls/lib/planner.py (long_mpc.py (MPC1, MPC2, MPC3))
  /data/params/d/dp_dynamic_gas (NG: dp (dragon) not op) dynamic_gas.py 裡的速度可做微調，
    特別注意這裡的單位是Km，另外一但開啟Dg, 原本的planner.py做微調已被Dg取代，
    但剎車可以再做微調,selfdrive/controls/lib/dynamic_gas.py (ShaneSmiskol)
  Driving.cc (Shen)

200819YP:
  float *lead = MDN_GROUP_SIZE*LEAD_MDN_N + SELECTION = 11*5+3 = 58
              = LONG_X_IDX - LEAD_IDX
  LEAD_MDN_N 5 // probs for 5 groups MDN = median = [(n+1)/2]th
  MDN_GROUP_SIZE 11 // 1 group = xyva + std (0~7) + score (now,2s,6s) (8~10) = 11
  SELECTION 3 //output 3 groups (lead now, in 2s and 6s) 3 time offsets
  MDN_VALS 4  //output xyva for each lead group

--- Part 3: Hack Car (***OP/PC/C2/CAN, PriusCAN, OP/PC/Panda/CAN)
  OP/OBD2/CANv, CAN1v, CAN2v, CAN3, Hack1v, OBD2v (OBD2PID), CAN4,   CAN5,DLC3,
  Prius1, P2, P3
  isotpGit, PyCAN, 2020

--- Part 4: Stop and Go (Port1, interface.py, smartDSU,)

210505: auto resume, Comma Pedal - Toyota

201231: Do Step 4-20 in port1 (Steps 1-3 Done -> /toyota/values.py).

201230: SmartDSU (buy 蝦皮購物台灣12/31; got it, installed it, tried on 1/4:
  Results: still cannot reaccelerate from 0 but the stopping from 50 kph to 0 by
    OP is better (smoother and larger distance from the lead car) than by the stock version;
    SDSU1, SDSU2): Hack ACC to enable Stop and Go on Siennas with TSS-P (TSS2)
    Q: Can SDSU reaccelerate from 0 automatically? Discord1, Discord2,
      Discord3 (need SnG hack or Pedal),
  Prius1
  A. SnG hack: /toyota/carcontroller.py
  # if CS.out.standstill and not self.last_standstill:
  # self.standstill_req = True
  -> reboot  (1/5: NG. Did 1, 2 in carcontroller081Liu.py: briskspirit

201208:
  TSS2 vehicles don't need for standstill request to be sent.
  If we send it PCM will go from status 8 to status 11 upon full stop (that where
    issue is from as we remove standstill request as soon as status != 8).
  Status 11 is a 3 seconds timer after which PCM goes into state 7 - standstill and
    asks to press resume or accelerator. After PCM is entered state 7 -
    standstill_req = False will not switch it back to cruise (8).
    JL: 8-cruise; 7-standstill; PCM: powertrain control module; OBD II)
  B. OP slows down for turns: controls/lib/planner.py
    v_curvature -> 1.625 -> 0.95
  C. lane change speed: controls/lib/pathplanner.py
  change line 20 from 45 to 40mph

201225: commaai openpilot stop and go
  I see from the code that there is a low speed lockout. Sometimes in the dbc files too.
    muhatashim commented on Oct 8, 2019
  Only the TSS2 Toyotas actually support stop and go natively.
  However, I actually made a commit just today on my fork that allows at
    least stopping and braking in stop and go traffic. Don't try it unless you
    really want that risk.
  selfdrive/car/toyota/interface.py (
  1. disable low speed lockout in order to allow braking and controlling in stop and go traffic.
  2. remove gas press no entry and return 0 acceleration when below 20mph.
  3. reduce actuator delay for smoother steering. NG: 12.25.)
  from cereal import car
  ButtonType = car.CarState.ButtonEvent.Type
  serialization: openpilot/cereal/car.capnp
  openpilot/cereal/services.py, service_list.yaml
  # These cars (CAR.PRIUS) use LQR/INDI
  Google: What is Adaptive Cruise Control with Stop-and-Go?

  comma pedal
  Pedal1,install1, install2, Flash1, 190519:  install1v (N/A), 190425: install2v (Wocsor)
  buy1, buy2, buy3

200627:
  People move to TSS 2.0 Toyotas and no longer need the Pedal.
  Only the TSS2 Toyotas actually support stop and go natively.
  Youtube: Openpilot: Stop and Go 0.6.6 Devel Accord/Bosch San Francisco Skyline
  Kegman fork: https://github.com/kegman/openpilot
  Installing a Fork of Openpilot with Workbench; /data/kegman.json is a file that holds
    parameters and is used on various branches / forks
  New! Timer for Nudgeless Auto Lane Change (default 2 seconds before lane change is made).

200602: Stop-and-Go Partly OK.
  Problems:
    1. OP cannot re-accelerate after stop. I have to resume it manually.
    2. OP is not or rarely using the brake, too fast for stopping.
    3. Not variable cruse speed (set to 45 km/h): approaching too fast to slow or
      stop lead car, too fast on curves.
    4. Lane centering is NG, too close to motor bikes or parked cars if No or
      invisible right lane markings on the far right lane.
--- comma two (C2A/C2B):
  C2A: 200428: Dongle ID 8bfda98c9c9e4291.
  C2B: 201111: Dongle ID b566aac108df59e3; Serial Number: 77c2722e.
--- Boot order of comma 2: C2BootOrder.txt

211001:
  Another C2B has the same problem but different issues
--- reboot 2 then get  “Choose Software to Install”
--- Select "Custom Software (Advanced)."
--- Enter https://openpilot.comma.ai and click "Install Software."
--- Search “Custom Software” in Discord says edit continue.sh
  root@localhost:/data$ find . continue.sh
  find: continue.sh: No such file or directory
  root@localhost:/data/data/com.termux/files$ ls
    home  tmp  usr
--- go to C2A
  root@localhost:/data/data/com.termux/files$ ls
    continue.sh  home  tmp	usr
--- FTP continue.sh and comma.sh from C2A to C2B
--- reboot
--- C2B OK!
--- OP079-081
    — C2 => Authorized SSH Keys => Edit => GitHub Username jinnliu
  jinn@Liu:~$ ssh root@10.0.0.7 -p 8022 -i c2key.pem
  root@localhost:/system/comma/home$

    Old steps not working for OP083+:
    1. copy and paste the following (Lines 1 to 28) to c2key.pem
    openpilot ssh key
    2. see ssh to Eon for the c2 IP: e.g. 10.0.0.13
    3. ssh to c2: Option 3 in openpilot/wiki/SSH
  jinn@Liu:~$ chmod 600 c2key.pem
  jinn@Liu:~$ ssh root@10.0.0.13 -p 8022 -i c2key.pem
--- we are now in c2
   scp ypreg.dlc to c2 (scp — secure copy (remote file copy program))
  jinn@Liu:~$ scp -i c2key.pem -P 8022 ./ypreg.dlc root@10.0.0.12:/data/openpilot/models/ypreg.dlc

210927:
  jinn@Liu:~$ ssh root@10.0.0.13 -p 8022 -i c2key.pem
  root@localhost:/data/openpilot/scripts$ ./update_now.sh
  root@localhost:/data/openpilot/scripts$ reboot
--- OK C2 now uses OP089 but …
  jinn@Liu:~$ ssh root@10.0.0.12 -p 8022 -i c2key.pem
    ssh: connect to host 10.0.0.12 port 8022: No route to host
--- Privacy Enhanced Mail (PEM) files are a type of Public Key Infrastructure (PKI)
  file used for keys and certificates. PEM, initially invented to make e-mail secure,
  is now an Internet security standard.
--- install Putty
  jinn@Liu:~$ sudo apt install putty
--- install PuTTYgen
  jinn@Liu:~$ sudo apt install putty-tools
--- Generate Key Pair for Authentication in Linux (PPK (Putty Private Key) )
  jinn@Liu:~$ puttygen -t rsa -b 2048 -C "user@host" -o c2key089.ppk
    Enter passphrase to save key: sxxxxxx6
--- do not use c2key089.ppk but use id_ed25519

211126: WB is not useful.
--- Install Workbench (Not necessary. WB1, it has more actions on C2.)
--- Download v0.1.5.linux.AppImage, cd ~/Downloads
  jinn@Liu:~/Downloads$ chmod +x Workbench-v0.1.5.linux.AppImage
  jinn@Liu:~/Downloads$ ./Workbench-v0.1.5.linux.AppImage
    Integrate the appimage into system? no

210925:
  comma two failed to install “Dashcam Software” (Continue to Setup ⇔ Dashcam
    Software: endless loops)
  root@localhost:/data$ mv openpilot openpilot081
  git clone https://github.com/commaai/openpilot.git --branch release2 --single-branch
--- check “disk free” in human readerable (-h)
  root@localhost:/data$ df -h
    /dev/block/sde19 2.8G  2.1G  667M  78% /system
    /dev/block/sda10  24G   17G  6.5G  73% /data
    /dev/fuse         24G   17G  6.5G  73% /mnt/runtime/default/emulated
    /dev/fuse         24G   17G  6.5G  73% /storage/emulated
    /dev/fuse         24G   17G  6.5G  73% /mnt/runtime/read/emulated
    /dev/fuse         24G   17G  6.5G  73% /mnt/runtime/write/emulated
--- remove all subdirectories and files in a directory
  root@localhost:/data/media/0/realdata$ rm -r *
  root@localhost:/data$ df -h

210925/1125:
--- reflash the NEOS operating system
--- https://github.com/commaai/eon-neos#restoring-on-linuxos-x
  jinn@Liu:~$ git clone https://github.com/commaai/eon-neos.git
  jinn@Liu:~$ cd eon-neos
--- Run ./download.py
  jinn@Liu:~/eon-neos$ ./download.py
--- failed 0925
  jinn@Liu:~/eon-neos$ ./download.py --master
--- failed 1125
--- Put your device into fastboot mode by turning off your device, then holding
    volume down + power.
--- Run ./flash.sh DO NOT DISCONNECT THE DEVICE!
  jinn@Liu:~/eon-neos$ ./flash.sh

201226: Install  ShaneSmiskol by PuTTY (NG: openpilot Unavailable)
  root@localhost:/data$ git clone -b stock_additions --single-branch https://github.com/shanesmiskol/openpilot --depth 1
  root@localhost:/data$ reboot
  Vim exit from insert mode pressing <ESC>
   1. cp values.py valuesX.py
   2. vim values.py
   3. delete: ctrl-v, move cursor, type d
   4. copy: right click mouse to copy
   5. paste: vim, type i (for insert), right click mouse to paste

200521: Install a fork OP to comma2
  Install ArnePilot by WB (OK but NG road test)
  cd /data;  mv openpilot openpilotOld  (rm -rf openpilot);
  git clone https://github.com/arne182/openpilot; cd openpilot; git checkout release4; reboot
  Kegmen: git clone https://github.com/kegman/openpilot; (NG road test)
  git checkout (one of the non-0.6 branches); reboot
  Konverter (supercombo.keras >> supercombo.py)
  Install PuTTY (on Windows)
  Install SSH on Windows
  Install PuTTY on Windows
  How to Use Putty with SSH Keys (comma2 IP: 10.0.0.?, Port: 8022, SSH Auth:
    openpilot.ppk, User Name: root)
  Comma2 IP: Touch WiFi -> Open WiFi Settings -> Connected (to your local WiFi)
  -> More Options -> “vertical” … -> Advanced -> IP address 10.0.0.?

200607: comma2 authentication key (c2key.ppk)
  How to transfer files to your EON
  Sometimes while testing or debugging issues you may want to manually transfer
    files to your EON. One easy way to do this is to use the popular FTP client Filezilla.
  Enable SSH. Click on "Settings" in OpenPilot and make sure the "Enable SSH"
    option is turned on.
  Next find the ip address of your EON. Under Settings > Wifi settings >
    Advanced (click on the 3 dots in the upper right hand corner) scroll all the
    way to the bottom to find IP. (Make sure your EON is on the same wifi as your computer)
  Create a file called openpilot.ppk and save the text at the bottom of this section
    in it. In Filezilla, go to your Site Manager and create a new entry for your EON
    and enter the IP address you found in the Host field. The port is 8022.
    The protocol should be SFTP. Logon Type will be "Key File". User is "root".
    Key file is the file you just created. Then hit connect.
  If you're having issues connecting, check to make sure you can ping your device on
    the IP you found. If ping doesn't work, then SSH won't either. You can try turning
    wifi off and then on again if you can't connect.
  All the settings listed above can be used to connect via PuTTY as well for command
    line access. You specify the keyfile in the settings under Connection -> SSH
    -> Auth -> Private key for authentication

PuTTY-User-Key-File-2: ssh-rsa
Encryption: none
Comment: imported-openssh-key
Public-Lines: 6
AAAAB3NzaC1yc2EAAAADAQABAAABAQC+iXXq30Tq+J5NKat3KWHCzcmwZ55nGh6W
ggAqECa5CasBlM9VeROpVu3beA+5h0MibRgbD4DMtVXBt6gEvZ8nd04E7eLA9LTZ
yFDZ7SkSOVj4oXOQsT0GnJmKrASW5KslTWqVzTfo2XCtZ+004ikLxmyFeBO8NOcE
rW1pa8gFdQDToH9FrA7kgysic/XVESTOoe7XlzRoe/eZacEQ+jtnmFd21A4aEADk
k00Ahjr0uKaJiLUAPatxs2icIXWpgYtfqqtaKF23wSt61OTu6cAwXbOWr3m+IUSR
UO0IRzEIQS3z1jfd1svgzSgSSwZ1Lhj4AoKxIEAIc8qJrO4uymCJ
Private-Lines: 14
AAABAQCEhXr8RxnaC92ecZMOqDuUkCjthsRHlYUczYJrvxwPqsfDq8qg+jtQlmON
N+5H7eolsZcIizncJ2tj9ubnlTNy8anUB9ikuA5pQsfpKuhcAoL9Ot30DzIQvS6V
opr2kEjxAu1VD40JaOLT2OrE02AVDodANYoUZv8e47irkAlosQqvAvw1ZwdV+Jho
/lt5yXOU8FSbYCW24ga6uj1q4bwf96ppMR0S+3VNkgW9ojURdSy2N9HScf3A+91A
yjR65a7I5N1CXNvTKePzJWnSr1JEajcJWMUrgLSVdJ2d/ohZC7N2nUkx3SaQpUHq
+OUedaxQ5VbA89mQaW/4UTUaBg7hAAAAgQDoDUSLEurtBg6HdxteHKHdbiSlW8D8
U98ofU4zLc404P9j05r4z/C2FbXBKhEvaf664hbk4CFMi8gSK+PpsBmxNPe3Cu86
u9i72vH1qWfinwGHK92N5biRLtPwU/miXoug63axM/dPUs64k865C86OUo8ogyHd
zdEU1ZOcbMw9rQAAAIEA0jNiPLxP3A2nrX0keKDI+VHuvOY88gdh0W5BuLMLovOI
Dk9aQFIbBbMuW1OTjHKv9NK+Lrw+YbCFqOGf1dU/UN5gSyE8lX/QFsUGUqUZx574
nJZnOIcy3ONOnQLcvHAQToLFAGUd7PWgP3CtHkt9hEv2koUwL4voikTP1u9Gkc0A
AACAPLxtzP7rcHi76uESO5e1O2/otgWo3ytjpszYv8boH3i42OpNrX0Bkbr+qaU4
3obY4trr4A1pIIyVID32aYq9yEbFTFIhYJaFhhxEzstEL3OQMLakyRS0w9Vs2trg
YpUlSBLIOmPNxonJIfnozphLGOnKNe0RWgGR8BnwhRYzu+k=
Private-MAC: 2af7f5a599fa35e22392b7770a2eb7a0be8718b7

comma2 Fingerprint
抓取车辆指纹2.0 https://openpilot.sdut.me/cn/fingerprints2.html

200428: Use WorkBench. Home2IP: 10.0.0.11   B4IP:
200428: fingerprint Good!

36: 8, 37: 8, 166: 8, 170: 8, 180: 8, 295: 8, 296: 8, 426: 6, 452: 8, 466: 8, 467: 8, 550: 8, 552: 4, 560: 7, 562: 6, 581: 5, 608: 8, 610: 8, 643: 7, 713: 8, 740: 5, 742: 8, 743: 8, 764: 8, 800: 8, 810: 2, 818: 8, 824: 8, 829: 2, 830: 7, 835: 8, 836: 8, 845: 5, 863: 8, 865: 8, 869: 7, 870: 7, 871: 2, 889: 8, 896: 8, 898: 8, 900: 6, 902: 6, 905: 8, 913: 8, 921: 8, 933: 8, 934: 8, 935: 8, 944: 8, 945: 8, 950: 8, 951: 8, 953: 8, 955: 8, 956: 8, 971: 7, 975: 5, 993: 8, 998: 5, 999: 7, 1000: 8, 1001: 8, 1002: 8, 1014: 8, 1017: 8, 1020: 8, 1041: 8, 1042: 8, 1044: 8, 1056: 8, 1057: 8, 1059: 1, 1071: 8, 1076: 8, 1077: 8, 1114: 8, 1161: 8, 1162: 8, 1163: 8, 1164: 8, 1165: 8, 1166: 8, 1167: 8, 1227: 8, 1235: 8, 1237: 8, 1264: 8, 1279: 8, 1541: 8, 1552: 8, 1553: 8, 1556: 8, 1557: 8, 1568: 8, 1570: 8, 1571: 8, 1572: 8, 1592: 8, 1593: 8, 1595: 8, 1777: 8, 1779: 8, 1786: 8, 1787: 8, 1788: 8, 1789: 8, 1792: 8, 1800: 8, 1872: 8, 1880: 8, 1904: 8, 1912: 8, 1937: 8, 1945: 8, 1953: 8, 1961: 8, 1968: 8, 1976: 8, 1990: 8, 1998: 8, 2015: 8, 2016: 8, 2024: 8
