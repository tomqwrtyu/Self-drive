'''
CAN 26a   2023.3.19, 7.18
for CAN 26, /cerealJLL/SConstruct  230714
/openpilot/zc2.py is run by /openpilot/zc1.py in CAN 26
'''
import platform
import subprocess
#p = subprocess.Popen(["python", "--help"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
  # The subprocess.Popen() creates a new process and execute the command list ["python", "--help"].
#output, errors = p.communicate()
  #print("#--- zc2.py output =\n", output)
arch = subprocess.check_output(["uname", "-m"], encoding='utf8').rstrip()
  # check_output is similar to run(), but it only returns the standard output of the command ["uname", "-m"],
  #   and raises a CalledProcessError exception if the return code is non-zero.
  # Uname stands for UNIX name. It is a utility to check the system information of your Linux computer.
print("#--- arch =", arch)
  #--- arch = x86_64
print("#--- platform.system() =", platform.system())
  #--- platform.system() = Linux
