#!/bin/bash
# JLL  2023.3.30, 4.6
# for 230330a, 230406
# from /OP091/tools/sim/launch_openpilot.sh
# see 230405 for old launch_openpilotJLL.sh
# . ~/sconsvenv/bin/activate
# (DP091venv) jinn@Liu:~/openpilot$ bash launch_openpilotJLL.sh
export PASSIVE="0"
export NOBOARD="1"
export SKIP_FW_QUERY="1"
export FINGERPRINT="HONDA CIVIC 2016"

export BLOCK="camerad,loggerd,encoderd,micd"

  #echo "#--- FINGERPRINT = " $FINGERPRINT  
  # one echo will result in all these export variables accesible to all processes in the Bash shell

  # handle pythonpath, from /OP091/launch_chffrplus.sh
ln -sfn $(pwd) /home/jinn/sconsvenv/bin
export PYTHONPATH="$PWD"
cd selfdrive/manager && ./manager.py
