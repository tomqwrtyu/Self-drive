CAN   JLL, 2021.5.2-5.10, 7.23
Copy and Paste Code Snippets to zc.py
--------------------------------------------------
CCCCCCCCCCCCCCCCCCCCCCCC Code Snippets
'''
CAN 15    2021.5.2-5.10
Hacking a real Toyota Prius car via comma two (comma2) by OP
Get Fingerprint from Prius

[1] http://www.nhcue.edu.tw/~jinnliu/teaching/AI17/port1.pdf
[2] How does openpilot work?
[3] Add support for your car to Comma.ai Openpilot
[4] Comma.ai Panda + Cabana Release
[5] https://my.comma.ai/cabana/?demo=1
[6] Communication with a Toyota Prius, 2009

Do CAN 13 first. Do NOT copy these to zc.py.
'''

---------- Step 0: Code Structure
commaai/opendbc/can/parser.py
  from opendbc.can.parser_pyx import CANParser, CANDefine

commaai/opendbc/can/parser_pyx.pyx
  from .common cimport CANParser as cpp_CANParser
    # default one dot in your current folder
  cdef class CANParser:
    cpp_CANParser *can
    const DBC *dbc
    self.dbc_name = dbc_name
    self.msg_name_to_address[name] = msg.address
    self.address_to_msg_name[msg.address] = name

--- from .some_module import some_class # .: same directory
    from ..some_package import some_function # ..: parent directory
    from . import some_class                # ...: grandparent directory
    these are relative imports vs absolute import
    from package1.module2 import function1
    import abc # abc can be a package or a module
    from abc import xyz # xyz can be a module, subpackage, or object, such as a class or function.

commaai/opendbc/can/parser.cc
  #include "common.h"
  CANParser::CANParser(int abus, const std::string& dbc_name,
    MessageState &state = message_states[op.address];
    state.address = op.address;
    state.size = msg->size;
  CANParser::CANParser(int abus, const std::string& dbc_name, bool ignore_checksum, bool ignore_counter)
  // Add all messages and signals
  for (int i = 0; i < dbc->num_msgs; i++) {
    const Msg* msg = &dbc->msgs[i];
    MessageState state = {
      .address = msg->address,
      .size = msg->size,
      const Signal *sig = &msg->sigs[j];
  for (int j = 0; j < msg->num_sigs; j++) {
    const Signal *sig = &msg->sigs[j];
    message_states[state.address] = state;

commaai/opendbc/can/common.h
  class MessageState {
  class CANParser {

---------- Step 1: Install Workbench (WB)

--- https://github.com/jfrux/workbench
--- Download v0.1.5.linux.AppImage
jinn@Liu:~$ cd ~/Downloads
jinn@Liu:~/Downloads$ chmod +x Workbench-v0.1.5.linux.AppImage
jinn@Liu:~/Downloads$ ./Workbench-v0.1.5.linux.AppImage
Integrate the appimage into system? no

--- read https://medium.com/@jfrux/introducing-workbench-for-openpilot-1bd60053dc9
--- connect PC and comma2 to same WiFi then open WB
ssh root@10.0.0.7 -p 8022 -i "/home/jinn/.ssh/openpilot_rsa"
jinn@Liu:~$ ssh root@10.0.0.7 -p 8022 -i "/home/jinn/.ssh/openpilot_rsa"
The authenticity of host '[10.0.0.7]:8022 ([10.0.0.7]:8022)' can't be established.
ECDSA key fingerprint is SHA256:5peozHRu3R3E8OzoS4dwhHBlWJX3v8EbjBq90zpCCEE.
Are you sure you want to continue connecting (yes/no)? yes
Warning: Permanently added '[10.0.0.7]:8022' (ECDSA) to the list of known hosts.
root@localhost:/data/openpilot$
--- logout comma2: "ctrl+d"
--- close WB

---------- Step 2: Install FileZilla

--- https://filezilla-project.org/download.php?type=client
--- https://medium.com/@jfrux/comma-eon-getting-connected-with-ssh-3ed6136e4a75
--- get comma2's Authentication Key: openpilot.ppk
    it is in Appendix 6 of C2Notes.docx
--- get comma2 IP: connect comma2 to a WiFi => tap on "WiFi"
    => "open WiFi settings", "More Option", "vertical three dots", "Advanced"
    => scroll down => IP adress: fe80::...; 10.0.0.7);
--- on FileZilla set
    Protocol: SFTP-SSH File Transfer Protocol; Logon Type: Key file;
    User: root; Key file: /home/jinn/OPFiles/openpilot.ppk; Click on "Connect"; Done.

---------- Step 3A: Get Fingerprint (prepare at home not in car)

--- https://github.com/commaai/openpilot/wiki/Fingerprinting
--- Fingerprinting 1.0
--- [1] http://www.nhcue.edu.tw/~jinnliu/teaching/AI17/port1.pdf
--- Step 1.2 in [1]: can0: Powertrain CAN Bus; can1: Radar CAN Bus
--- redo Steps 1-3 [1]: Done on 2020.4.8. See C2Notes.docx.

--- run comma2 at home not in car
--- Step 2.2: open WB => click on ">"
root@localhost:/data/openpilot$ tmux ls
comma: 1 windows (created Wed Dec 31 19:00:38 1969)
root@localhost:/data/openpilot$ logout
Connection to 10.0.0.7 closed.
jinn@Liu:~$
--- click on "<" then on ">"
root@localhost:/data/openpilot$ tmux a
--- now in 0:bash*
...
thermald logmessaged ui uploader deleter updated logcatd tombstoned sensord pandad
...
{"event": "STATUS_PACKET", "count": 480, "health": null, "location": null,
"thermal": {"thermal": {"cpu0DEPRECATED": 0, "cpu1DEPRECATED": 0, "cpu2DEPRECATED": 0,
"cpu3DEPRECATED": 0, "memDEPRECATED": 0, "gpuDEPRECATED": 0, "batDEPRECATED": 0,
"freeSpace": 0.21143551170825958, "batteryPercent": 91, "batteryStatus": "Discharging",
"fanSpeed": 0, "started": false, "usbOnline": false, "startedTs": 0, "thermalStatus":
"green", "batteryCurrent": 248106, "batteryVoltage": 4250768, "chargingError": false,
"chargingDisabled": false, "memUsedPercent": 25, "cpuPerc": 10, "pa0DEPRECATED": 0,
"networkType": "wifi", "offroadPowerUsage": 0, "networkStrength": "great", "carBatteryCapacity": 18432950,
"cpu": [35.79999923706055, 36.099998474121094, 35.79999923706055, 35.099998474121094],
"gpu": [33.70000076293945], "mem": 37.0, "bat": -29.0, "ambient": 31.0}, "logMonoTime": 658579610633, "valid": true}}
...
"chargingDisabled": false, "memUsedPercent": 25, "cpuPerc": 6, "pa0DEPRECATED": 0,
"gpu": [34.70000076293945], "mem": 37.400001525878906, "bat": -29.0, "ambient": 32.0}, "logMonoTime": 1112796852751, "valid": true}}
...
--- type ctrl+c+d to kill the openpilot processes

--- reconnect comma2
root@localhost:/data/openpilot$ tmux a
--- type ` (backtick) and then c
--- now in bash 1, logout to experimental
root@localhost:/$ logout
--- close the terminal to exit bash 0

---------- Step 3B: Get Fingerprint (in car but Failed!)

--- install comma2 to car; connect comma2 and PC to same Wifi
--- turn "on" the car
--- Step 2.4
root@localhost:/data/openpilot$ tmux a
--- now in bash 0
...
thermald logmessaged ui uploader deleter updated logcatd tombstoned sensord pandad controlsd plannerd loggerd radard calibrationd paramsd camerad modeld proclogd locationd clocksd ubloxd dmonitoringd dmonitoringmodeld gpsd rtshield
...
to upload ('2021-05-08--02-04-59--13/qlog.bz2', '/data/media/0/realdata/2021-05-08--02-04-59--13/qlog.bz2')
{"event": "upload", "key": "2021-05-08--02-04-59--13/qlog.bz2", "fn": "/data/media/0/realdata/2021-05-08--02-04-59--13/qlog.bz2", "sz": 764758}
checking '2021-05-08--02-04-59--13/qlog.bz2' with size 764758
uploading '/data/media/0/realdata/2021-05-08--02-04-59--13/qlog.bz2'
upload_url v1.3 https://commadata2.blob.core.windows.net/qlog/b566aac108df59e3/2021-05-08--02-04-59/13/qlog.bz2?se=2021-05-15T06%3A19%3A06Z&sp=cw&sv=2018-03-28&sr=b&sig=5cfxG7Wo5gW2tbFsCuocmn4whSPKIqYpEN0xfeh6VWM%3D {'x-ms-blob-type': 'BlockBlob'}
{"event": "STATUS_PACKET", "count": 354240, "health": {"health": {"voltage": 13293, "current": 0, "ignitionLine": true, "controlsAllowed": false, "gasInterceptorDetected": false, "startedSignalDetectedDeprecated": false, "hasGps": true, "canSendErrs": 198142, "canFwdErrs": 0, "gmlanSendErrs": 0, "hwType": "uno", "fanSpeedRpm": 3345, "usbPowerMode": "cdp", "ignitionCan": false, "safetyModel": "noOutput", "faultStatus": "none", "powerSaveEnabled": false, "uptime": 177497, "faults": [], "canRxErrs": 0}, "logMonoTime": 177385939664779, "valid": true}, "location": null, "thermal": {"thermal": {"cpu0DEPRECATED": 0, "cpu1DEPRECATED": 0, "cpu2DEPRECATED": 0, "cpu3DEPRECATED": 0, "memDEPRECATED": 0, "gpuDEPRECATED": 0, "batDEPRECATED": 0, "freeSpace": 0.21034909784793854, "batteryPercent": 100, "batteryStatus": "Charging", "fanSpeed": 52, "started": true, "usbOnline": false, "startedTs": 177264821997689, "thermalStatus": "green", "batteryCurrent": 1243584, "batteryVoltage": 4216589, "chargingError": false, "chargingDisabled": false, "memUsedPercent": 43, "cpuPerc": 66, "pa0DEPRECATED": 0, "networkType": "wifi", "offroadPowerUsage": 0, "networkStrength": "good", "carBatteryCapacity": 30007289, "cpu": [67.0, 63.79999923706055, 66.0999984741211, 66.4000015258789], "gpu": [64.9000015258789], "mem": 64.80000305175781, "bat": 0.0, "ambient": 43.0}, "logMonoTime": 177385942168789, "valid": true}}
{"event": "upload_success", "key": "2021-05-08--02-04-59--13/qlog.bz2", "fn": "/data/media/0/realdata/2021-05-08--02-04-59--13/qlog.bz2", "sz": 764758}
upload done, success=True
...
--- type ` (backtick) and c
--- now in bash 1
root@localhost:/data/openpilot$ tmux ls
comma: 2 windows (created Wed Dec 31 19:00:38 1969) (attached)
root@localhost:/data/openpilot$ ls
root@localhost:/data/openpilot$ export PYTHONPATH=data/openpilot/selfdrive/debug/get_fingerprint.py
--- make sure the script is running with the car on for at least
    15 consecutive seconds
--- Error: No response. Failed.

---------- Step 3C: Get Fingerprint (in car OK!)

--- Fingerprinting 1.0
--- change /openpilot/selfdrive/car/toyota/values.py to values075Liu.py
--- turn "off" the car
root@localhost:/data/openpilot$ tmux a
no sessions
root@localhost:/data/openpilot$ reboot
packet_write_wait: Connection to 192.168.1.75 port 8022: Broken pipe
root@localhost:/data/openpilot$ tmux a
--- now in bash 0

--- type ` (backtick) and c
--- now in bash 1
root@localhost:/data/openpilot$ PYTHONPATH=/data/openpilot PREPAREONLY=1 /data/openpilot/selfdrive/debug/get_fingerprint.py
--- turn on the car's ignition, and wait up to ~20 seconds
    to ensure all the appropriate DBC messages are seen, like this...
--- get the printed text, a dictionary-like structure, as follows
--- ctrl+c
--- Copy and paste the printed text to here as

--- new fingerprint on 2021.5.10
number of messages 130:
fingerprint 36: 8, 37: 8, 166: 8, 170: 8, 180: 8, 295: 8, 296: 8, 426: 6, 452: 8, 466: 8, 467: 8, 550: 8, 552: 4, 560: 7, 562: 6, 581: 5, 608: 8, 610: 8, 643: 7, 713: 8, 740: 5, 742: 8, 743: 8, 764: 8, 800: 8, 810: 2, 818: 8, 824: 8, 829: 2, 830: 7, 835: 8, 836: 8, 845: 5, 863: 8, 865: 8, 869: 7, 870: 7, 871: 2, 889: 8, 896: 8, 898: 8, 900: 6, 902: 6, 905: 8, 913: 8, 921: 8, 933: 8, 934: 8, 935: 8, 944: 8, 945: 8, 950: 8, 951: 8, 953: 8, 955: 8, 956: 8, 971: 7, 975: 5, 993: 8, 998: 5, 999: 7, 1000: 8, 1001: 8, 1002: 8, 1014: 8, 1017: 8, 1020: 8, 1041: 8, 1042: 8, 1044: 8, 1056: 8, 1057: 8, 1059: 1, 1071: 8, 1076: 8, 1077: 8, 1114: 8, 1161: 8, 1162: 8, 1163: 8, 1164: 8, 1165: 8, 1166: 8, 1167: 8, 1227: 8, 1235: 8, 1237: 8, 1264: 8, 1279: 8, 1541: 8, 1552: 8, 1553: 8, 1556: 8, 1557: 8, 1568: 8, 1570: 8, 1571: 8, 1572: 8, 1592: 8, 1593: 8, 1595: 8, 1777: 8, 1779: 8, 1786: 8, 1787: 8, 1788: 8, 1789: 8, 1792: 8, 1800: 8, 1872: 8, 1880: 8, 1904: 8, 1912: 8, 1937: 8, 1945: 8, 1953: 8, 1961: 8, 1968: 8, 1976: 8, 1988: 8, 1990: 8, 1996: 8, 1998: 8, 2000: 8, 2001: 8, 2008: 8, 2015: 8, 2016: 8, 2017: 8, 2024: 8

--- old fingerprint on 2020.4.28
fingerprint 36: 8, 37: 8, 166: 8, 170: 8, 180: 8, 295: 8, 296: 8, 426: 6, 452: 8, 466: 8, 467: 8, 550: 8, 552: 4, 560: 7, 562: 6, 581: 5, 608: 8, 610: 8, 643: 7, 713: 8, 740: 5, 742: 8, 743: 8, 764: 8, 800: 8, 810: 2, 818: 8, 824: 8, 829: 2, 830: 7, 835: 8, 836: 8, 845: 5, 863: 8, 865: 8, 869: 7, 870: 7, 871: 2, 889: 8, 896: 8, 898: 8, 900: 6, 902: 6, 905: 8, 913: 8, 921: 8, 933: 8, 934: 8, 935: 8, 944: 8, 945: 8, 950: 8, 951: 8, 953: 8, 955: 8, 956: 8, 971: 7, 975: 5, 993: 8, 998: 5, 999: 7, 1000: 8, 1001: 8, 1002: 8, 1014: 8, 1017: 8, 1020: 8, 1041: 8, 1042: 8, 1044: 8, 1056: 8, 1057: 8, 1059: 1, 1071: 8, 1076: 8, 1077: 8, 1114: 8, 1161: 8, 1162: 8, 1163: 8, 1164: 8, 1165: 8, 1166: 8, 1167: 8, 1227: 8, 1235: 8, 1237: 8, 1264: 8, 1279: 8, 1541: 8, 1552: 8, 1553: 8, 1556: 8, 1557: 8, 1568: 8, 1570: 8, 1571: 8, 1572: 8, 1592: 8, 1593: 8, 1595: 8, 1777: 8, 1779: 8, 1786: 8, 1787: 8, 1788: 8, 1789: 8, 1792: 8, 1800: 8, 1872: 8, 1880: 8, 1904: 8, 1912: 8, 1937: 8, 1945: 8, 1953: 8, 1961: 8, 1968: 8, 1976: 8, 1990: 8, 1998: 8, 2015: 8, 2016: 8, 2024: 8
--- 36: 8 (key: bytes); key: address of message; bytes: length of message

 0:bash-   1:bash*   10/05  00:13:09
root@localhost:/data/openpilot$ logout
 0:bash*  10/05  00:13:55
--- OK. Done.

---------- Step 3C: Get Fingerprint
--- Fingerprinting 2.0
--- Step 1: Record a Drive
--- Step 2: Log into useradmin: https://my.comma.ai/useradmin/
    comma useradmin
--- Step 3: Find the firmware
    openpilot/selfdrive/debug/fingerprint_from_route.py
    openpilot/tools/lib/route.py
--- Step 4: Add new firmware versions
--- Error: comma2 (dongle_id: ) not yet paired, not in useradmin.

---------- Step 3D: Get Fingerprint
--- Fingerprinting 2.0 (Alternative)
--- turn "off" the car
root@localhost:/data/openpilot$ cd selfdrive/boardd
root@localhost:/data/openpilot/selfdrive/boardd$ pkill ./manager.py
root@localhost:/data/openpilot/selfdrive/boardd$ ./boardd
--- ctrl+c
root@localhost:/data/openpilot/selfdrive/boardd$ tmux a
--- now in bash 0

--- type ` (backtick) and c
--- now in bash 1
root@localhost:/data/openpilot$ cd selfdrive/car
root@localhost:/data/openpilot/selfdrive/car$ python fw_versions.py
Getting vin...
--- turn on the car, when message shows getting Vin
    then run python fw_versions.py again.
--- ctrl+c
root@localhost:/data/openpilot/selfdrive/car$ python fw_versions.py
Getting vin...
--- Error: No response. Failed.

---------- the following failed and is not needed
----- the following was not needed
root@localhost:/data/openpilot$ cd selfdrive/boardd
root@localhost:/data/openpilot/selfdrive/boardd$ ./boardd
selfdrive/boardd/boardd.cc: starting boardd
selfdrive/boardd/boardd.cc: attempting to connect
selfdrive/boardd/boardd.cc: fw signature: 8045f4487d396190
selfdrive/boardd/boardd.cc: panda serial: 9470af371a7fc384
selfdrive/boardd/boardd.cc: connected to board
selfdrive/boardd/panda.cc: Receive buffer full
--- Switch between sessions in tmux by ctrl-b s
root@localhost:/data/openpilot/selfdrive/boardd$ logout

----- the following was not needed
root@localhost:/data/openpilot$ ./launch_openpilot.sh | grep "fingerprinted"
/data/openpilot/launch_chffrplus.sh: line 39: echo: write error: No such device
--- type ctrl+c twice
--- Error on comma2: camera melfuction
--- unplug and replug the power of comma2

CCCCCCCCCCCCCCCCCCCCCCCC
'''
CAN 14    2021.5.2
Convert Little and Big Endian Hex String to Int in Python
https://www.delftstack.com/howto/python/python-hex-to-int/
(OP) jinn@Liu:~/openpilot/acan$ python zc.py
'''
#big_endian = 'efbe'
#little_endian = '0BB8' # RPM value CAN 13 [3] Part II
#little_endian = '0000000BB8' # CAN 13 [3] Part II
#little_endian = '0000003812' # throttle value not speed, CAN 13, 90 MPH
little_endian = '0000004B70' # CAN 13, 120 MPH
def to_little(val):
  little_hex = bytearray.fromhex(val)
  little_hex.reverse()
  print("Byte array format:", little_hex)
  str_little = ''.join(format(x, '02x') for x in little_hex)
  return str_little

#little_endian = to_little(big_endian)
print("Little endian hex:", little_endian)
a = int(little_endian, 16)
print("Hex to int:", a)
#speed = a/3600
#print("Speed km/h:", speed) # no not speed value

--------------------------------------------------
----- Appendix
--- Toyota CAN Buses: Powertrain CAN, ADAS CAN,
    CAN/LIN/GMLAN buses, CAN, BEAN (Body Electronics Area Network),
    AVC-LAN (Audio Visual Communication-Local Area Network)
--- Honda: big endianness, vehicle and radar CANs

----- Abbreviation
ECU: electronic control unit
EPS: electrical power steering controller
FW: Firmware
ISO-TP: Transport Protocol
PCM: power-train control module
slcan: serial line CAN
UDS: Unified Diagnostic Services
VIN: Vehicle Identification Number
