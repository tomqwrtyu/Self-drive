CB 1     2023.1.27 - 2.10
Code Overview: From Ignition ON to First comma body (CB) Move

CAN 24   2023.2.15
for class LatControl(ABC)

CAN 23   2022.12.15
for /home/jinn/YPN/tinygrad

CAN 22   2022.4.20
for /home/jinn/YPN/Leon/parserB5.py

CAN 21   2022.1.26
Notes: datagenB6.py, train_modelB6.py

CAN 20   2021.7.23-8.2, 9.10
Read YP's Code in OP079C2
Read zcCS3a.py first

CAN 19   2021.7.23, 9.10
Run model opEffPN.dlc by uiview.py on comma two (c2) with OP079

CAN 18   2021.5.11-5.25, 6.6
Do CAN 15 first in zcCS5.py
Code Overview: All files producing the fingerprint output in CAN 15.

CAN 17   2021.5.9
??? result():
openpilot/selfdrive/car/fingerprints.py
openpilot/selfdrive/debug/show_matching_cars.py

CAN 16   2021.5.8
??? ->: def new_message(service: Optional[str] = None, size: Optional[int] = None)
          -> capnp.lib.capnp._DynamicStructBuilder: ...     in
cereal/messaging/__init__.py     called by
openpilot/selfdrive/debug/get_fingerprint.py
  CAN 15    ---------- Step 3: Get Fingerprint

CAN 15   2021.5.2-5.10
Hacking a real Toyota Prius car via comma two (comma2) by OP
Get Fingerprint from Prius

CAN 14   2021.5.2
Convert Little and Big Endian Hex String to Int in Python
(OP) jinn@Liu:~/openpilot/acan$ python zc.py

CAN 13   2021.4.29-5.2
Hacking a "virtual" car

CAN 12   2021.4.26
openpilot/selfdrive/debug/internal/check_alive_valid.py
(OP) jinn@Liu:~/openpilot/acan$ python zc.py

CAN 11b  2021.4.26
https://github.com/commaai/cereal
--- Terminal 1
(OP) jinn@Liu:~/openpilot/acan$ python zc1.py
--- Terminal 2
(OP) jinn@Liu:~/openpilot/acan$ python zc.py

CAN 11a  2021.4.26
https://github.com/commaai/cereal
--- Terminal 1
(OP) jinn@Liu:~/openpilot/acan$ python zc1.py

CAN 10d  2021.4.21-4.27, 6.20-6.22, 6.30-7.1
Code Overview: From Neural Net to CAN by Steering

CAN 10c  2021.4.21-4.27, 6.14-6.22, 30
Code Overview: From Neural Net to CAN by Steering

CAN 10b  2021.6.15-6.22, 29
Code Overview: From Neural Net to CAN by Steering

CAN 10a  2021.4.21-4.27, 5.27-6.24, 30
Code Overview: From Neural Net to CAN by Steering

CAN 9    2021.4.20, 4.27, 5.6
jinn@Liu:~/can$ python zc.py
(OP) jinn@Liu:~/openpilot/opendbc/can$ python dbc.py

CAN 8    Do CAN 3 first
jinn@Liu:~/can$ python zc.py
Input: /home/jinn/can/motohawk.dbc

CAN 7    Do CAN 3 first
--- Terminal 1
jinn@Liu:~/can$ candump vcan0 | python3 -m cantools decode motohawk.dbc
--- Terminal 2
jinn@Liu:~/can$ python zc.py
Input: /home/jinn/can/motohawk.dbc

CAN 6    Do CAN 3 first
--- Terminal 1
jinn@Liu:~/can$ python zc.py listen vcan0
--- Terminal 2
jinn@Liu:~/can$ python zc.py send vcan0 1ff deadbeef

CAN 5    Do CAN 3 first
--- Terminal 1
jinn@Liu:~/can$ candump vcan0
--- Terminal 2
jinn@Liu:~/can$ python zc.py
Input: /home/jinn/can/motohawk.dbc
Output: in Terminals

CAN 4    Do CAN 3 first
https://github.com/eerimoq/cantools

CAN 3    2021.4.16, 4.29
https://github.com/eerimoq/cantools

CAN 2
openpilot/selfdrive/car/fw_versions.py

CAN 1
openpilot/selfdrive/test/test_models.py
