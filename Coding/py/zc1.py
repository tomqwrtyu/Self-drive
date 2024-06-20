'''
CAN 26   2023.3.19
for tools/replay/can_replay.py
    230319: UI: PC+C2: "loggerd": JLL #subprocess.call
from An Introduction to Python Subprocess: Basics and Examples

(sconsvenv) jinn@Liu:~/openpilot$ python zc1.py
zc1.py runs zc2.py in CAN 26a
'''
import os
import subprocess
from common.basedir import BASEDIR
  # Example 1: Running shell commands:
#result = subprocess.run(["dir"], shell=True, capture_output=True, text=True)
  # subprocess.run() runs a subprocess with the command "dir".
  # ["dir"] == jinn@Liu:~/openpilot/aJLL/Coding/py$ dir
  #print("#--- zc.py output 1 =\n", result.stdout)
  # Example 2: Running another Python script zc2.py
result = subprocess.run(["python", "zc2.py"], capture_output=True, text=True)
print("#--- zc.py running zc2.py result:\n", result.stdout)
  # Example 3: Running the executable file selfdrive/loggerd/bootlog
subprocess.call("./bootlog", cwd=os.path.join(BASEDIR, "selfdrive/loggerd"))
  # call() runs "./bootlog" (a process) in selfdrive/loggerd/ and waits for it to complete.
  # Output: selfdrive/loggerd/bootlog.cc: bootlog to /home/jinn/.comma/media/0/realdata/boot/2023-03-19--19-19-44
  #--- BASEDIR = /home/jinn/openpilot
