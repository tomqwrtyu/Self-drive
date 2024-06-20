CAN   JLL, 2021.4.16 - 2023.10.20
Copy and Paste Code Snippets in zcCS*.py to zc.py and Run.
For reading, checking, learning, and running openpilot (OP) code

Do CAN 10a-d in zcCS3a-d.py: Code Overview: From Neural Net to CAN by Steering
Do CAN 18    in zcCS6.py:    Code Overview for CAN 15
Do CAN 15    in zcCS5.py:    Get Fingerprint from Toyota Prius
Do CAN  9    in zcCS2.py:    Run dbc.py

Read the following code in order:
1. /selfdrive/modeld/modeld.cc
2. /selfdrive/modeld/models/driving.cc
3. /cereal/messaging/messaging.hpp
4. /selfdrive/controls/lib/pathplanner.py
5. /selfdrive/controls/controlsd.py
6. /selfdrive/boardd/boardd.cc

--------------------------------------------------
230109: OP Install
  jinn@Liu:~/OP090$ git clone https://github.com/commaai/body.git
  jinn@Liu:~/OP090$ git clone https://github.com/commaai/cereal.git
  jinn@Liu:~/OP090$ git clone https://github.com/commaai/laika.git
  jinn@Liu:~/OP090$ git clone https://github.com/commaai/opendbc.git
  jinn@Liu:~/OP090$ git clone https://github.com/commaai/panda.git
  jinn@Liu:~/OP090$ git clone https://github.com/commaai/rednose.git
  jinn@Liu:~/OP090$ git clone https://github.com/geohot/tinygrad.git

221214: Install OP090 to study tinygrad
  jinn@Liu:~$ git clone https://github.com/commaai/openpilot.git OP090
  (YPN) jinn@Liu:~/YPN$ git clone https://github.com/geohot/tinygrad.git tinygrad
  (YPN) jinn@Liu:~/YPN/tinygrad$ python jl1.py
--- How to Remove Snap Packages in Ubuntu Linux
  jinn@Liu:~$ snap list
  jinn@Liu:~$ sudo snap remove --purge snap-store
--- clean disk
  jinn@Liu:~$ du -sh /var/cache/apt/archives
  jinn@Liu:~$ sudo apt-get clean
  jinn@Liu:~$ sudo apt-get autoremove --purge

211228: We use OP082 for writing OP code to run on comma two with OP079.
--- download openpilot v0.8.2 to PC
    https://drive.google.com/file/d/1gxOXM5JA8pNNlvqy5K8qt8uHqEBIwv2F/view?usp=sharing
--- the following does not have /tools
  jinn@Liu:~$ git clone https://github.com/commaai/openpilot --branch v0.8.2 --single-branch

210905: A3. Q3 is solved (see train_modelA1.py).
210830: Q3. Convert opEffA2.py (train_modelA.py) to opeffA2.h5 for Leon's main.py to run opEffA2.h5. See train_modelA.py and modelA.py for my errors. Idea: Pass constant args to class MBConvBlock(args).
210921: A2. See Line 79 in main.py
210818: Q1. Where is QCOM defined? Q2. (1, 12?, 128?, 256) in output.txt

--------------------------------------------------
----- Appendix 1: Python Tutorials
--- Python: Object Oriented Programming
    https://www3.ntu.edu.sg/home/ehchua/programming/webprogramming/Python1a_OOP.html
--- Understanding Object Instantiation and Metaclasses in Python
    https://www.honeybadger.io/blog/python-instantiation-metaclass/
--- Python's Instance, Class, and Static Methods Demystified
--- The Basics of NumPy Arrays (Tensors)
    https://jakevdp.github.io/PythonDataScienceHandbook/02.02-the-basics-of-numpy-arrays.html
    https://vitalflux.com/tensor-explained-with-python-numpy-examples/
--- Why you should avoid using Python Lists?

----- Appendix 2: C++ Tutorials
--- C++ Programming Language
    https://www.geeksforgeeks.org/c-plus-plus/
--- C/C++ Data Types
    http://www.nhcue.edu.tw/~jinnliu/teaching/nde10f/cs11.htm
    https://www.geeksforgeeks.org/c-data-types/
    https://os.mbed.com/handbook/C-Data-Types
    https://www.tutorialspoint.com/cplusplus/cpp_data_types.htm
--- C/C++ Memory Layout, Pointers
    https://www.tutorialspoint.com/cplusplus/cpp_quick_guide.htm
    https://www.geeksforgeeks.org/memory-layout-of-c-program/
    https://www.cplusplus.com/doc/tutorial/pointers/
    https://www.geeksforgeeks.org/smart-pointers-cpp/
--- C/C++ Scope resolution operator ::, Template Library
    ::identifier; class-name::identifier; namespace::identifier
    https://docs.microsoft.com/en-us/cpp/cpp/scope-resolution-operator?view=msvc-160&viewFallbackFrom=vs-2017
    https://www.boost.org/sgi/stl/stl_introduction.html

----- Appendix 3: OP & commaai publications
  2021.10.12 How openpilot works in 2021
  2021.03.29 End-to-end lateral planning
  2020.11.19 How to spend 1,000x more money on self-driving and still lose
  2020.11.16 Active Driving Assistance Systems: - Consumer Reports
  2020.03.11 From Vision To Architecture: How to use openpilot and live
  2019.12.03 A Tour Through openpilot
  2018.01.30 openpilot port guide for Toyota models
    http://www.nhcue.edu.tw/~jinnliu/teaching/AI17/port1.pdf
  2017.11.29 Add support for your car to Comma.ai Openpilot
  2017.06.26 How does openpilot work? wiki openpilot

----- Appendix 4: comma2
--- FTP Data: /data/media/0/realdata

----- Appendix: Car
--- Toyota CAN Buses: Powertrain CAN, ADAS CAN,
    CAN/LIN/GMLAN buses, CAN, BEAN (Body Electronics Area Network),
    AVC-LAN (Audio Visual Communication-Local Area Network)
--- Honda: big endianness, vehicle and radar CANs
--- Kalman Filter
    https://www.kalmanfilter.net/default.aspx
    https://github.com/sshane/kalman-filters

----- Appendix 5: Github
--- Github: Be very careful to click on "Discard" in Atom. You may loss your code!!!
--- Github: Duplicating a repository (from CAN to CAN1)
  jinn@Liu:~/tmp$ git clone --bare https://github.com/JinnAIGroup/CAN.git CAN1.git
    Cloning into bare repository 'CAN1.git'...
    Username for 'https://github.com': jinnliu
    Password for 'https://jinnliu@github.com':
    - go to JinnAIGroup, make sure you are logged in, and create a new (blank) repo CAN1
  jinn@Liu:~/tmp/CAN1.git$ git push --mirror https://github.com/JinnAIGroup/CAN1.git
    - go to /JinnAIGroup and check the new (full) repo CAN1
    - trash /tmp/CAN1.git
    - Delete /JinnAIGroup/CAN1
--- Github: Pull & Push
    - go to JinnAIGroup and create a new (blank) repo OP082
    - clone the repo to my PC
  jinn@Liu:~/tmp$ git clone https://github.com/JinnAIGroup/OP082
    Cloning into 'OP082'...
    remote: Counting objects: 100% (3/3), done.
    - https://happygitwithr.com/push-pull-github.html
  jinn@Liu:~/tmp/OP082$ git remote show origin
  jinn@Liu:~/tmp/OP082$ echo "A line I wrote on my local computer" >> README.md
  jinn@Liu:~/tmp/OP082$ git status
    	modified:   README.md
    - stage (“add”) this change to your remote repo on GitHub.
  jinn@Liu:~/tmp/OP082$ git add -A
    - commit this change to your remote repo on GitHub.
  jinn@Liu:~/tmp/OP082$ git commit -m "A commit from my local computer"
    [main 1f3d87a] A commit from my local computer
     1 file changed, 1 insertion(+), 1 deletion(-)
    - push to your remote repo on GitHub.
  jinn@Liu:~/tmp/OP082$ git push
    To https://github.com/JinnAIGroup/OP082
       6c997ae..1f3d87a  main -> main
    - go to JinnAIGroup and see the modified README.md

----- Appendix 6: Modified and Run Code by JLL
  dbcX.py etc. is the original file and dbc.py has been modified.
  2021.03.20 openpilot/selfdrive/test/process_replay/process_replay.py
  2021.03.26 openpilot/selfdrive/ui/ui.cc
  2021.04.20 openpilot/opendbc/can/dbcX.py
  2021.04.21 openpilot/cereal/car.capnp
  2021.04.21 openpilot/selfdrive/controls/lib/latcontrol_pidX.py
  2021.04.21 openpilot/selfdrive/controls/controlsdX.py
  2021.04.27 openpilot/selfdrive/test/longitudinal_maneuvers/test_longitudinal.py
  2021.05.10 openpilot/selfdrive/debug/show_matching_carsX.py
  2021.05.11 openpilot/selfdrive/debug/get_fingerprintX.py
  2021.05.22 /OP082/selfdrive/boardd/boardd.cc
  2021.05.22 /OP082/selfdrive/boardd/panda.cc
  2021.05.25 /OP082/cereal/messaging/messaging.cc
  2021.05.22 /OP082/cereal/messaging/impl_zmq.cc

----- Appendix 7: Miscellany
--- Which is better? h5 vs npy to store arrays
    https://blog.csdn.net/reform513/article/details/105861914
--- Atom Editor: https://atom.io/
    Edit > Preferences > Packages > language-python > Tab Length > Default: 2
    Edit > Stylesheet and then paste:
    .tree-view {
      font-size: 16px;}
--- change file/folder mode
    jinn@Liu:~/tmp$ sudo chmod a+rwx home

----- Abbreviations with Tutorials

ABC: ABC in Python (Abstract Base Class)
acado: ACADO Toolkit: an algorithm collection for automatic control and dynamic optimization
acados: a modular open-source framework for fast embedded optimal control
  The class AcadosOcp can be used to formulate optimal control problems (OCP), for
    which an acados solver (AcadosOcpSolver) can be created.
  acados is the successor of ACADO and has been completely rewritten such that code
    generation is not necessary anymore. In this way the code is much easier to maintain,
    extend and debug.
adb: Android Debug Bridge
AGNOS is the Ubuntu-based operating system for your comma three devkit.
ALKA: Active Lane Keeping Assist: A device assists the driver to keep vehicle in lane.
ARM: Advanced RISC Machine
BLDC: Brushless DC motor: See Apdx 2, FOC, body/board/bldc/bldc.c
BLOB (blob): “Binary Large Object,” a data type that stores binary data.
CAN: control area network (for cars). CAN-TX, CAN-RX: http://wiki.csie.ncku.edu.tw/embedded/CAN
CFFI: a package providing a C Foreign Function Interface for Python
capnp: Cap’n Proto: a data serialization format for exchanging data between computer programs
  Cap’n Proto Schema Language, Cap’n Proto Example Usage, Cap’n Proto Quickstart
  kj::ArrayPtr: How to get byte[] into capnp::Data
    kj::ArrayPtr is fundamentally a pair of a pointer and a size.
      Blobs: Text, Data; Text is always UTF-8 encoded and NUL-terminated
      Data is a completely arbitrary sequence of bytes.
        byte buffer[256];
        kj::ArrayPtr<byte> bufferPtr = kj::arrayPtr(buffer, sizeof(buffer));
        byte* ptr = bufferPtr.begin();
        size_t size = bufferPtr.size();
        struct Tiding { id @0 :Text; payload @1 :Data; }
        setPayload(kj::arrayPtr(data, sizeof(data)))
  Proper syntax to read and write byte array in Cap'n Proto?
  Cap’n Proto is built on top of a basic utility library called KJ – KJ is simply
    the stuff which is not specific to Cap’n Proto serialization.
    For now, the two are distributed together. The name “KJ” has no particular meaning;
    it was chosen to be short and easy-to-type.
CRC: Cyclic Redundancy Check is a checksum algorithm to detect inconsistency
  of data, e.g. bit errors during data transmission.
CS: code snippet
Cython: cython Getting started with cython
dbc: CAN dada base container
dlc: deep learning container
DSP: digital signal processor
Eigen: C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms
ECU: electronic control unit
EPS: electrical power steering controller
ESP: electronic stability program (ESP Electronic Stability Program Explained)
ESR: Delphi Electronically Scanning Radar (ESR), ESR.dbc (_ONCOMING, oncoming, log.capnp), 2020 (Auto) (EcoCAR) (Thesis)
FOC: Field Oriented Control (FOC) - A Deep Dive. See Apdx 2, BLDC
FTP: file transfer protocol
FW: firmware
GNSS: Global navigation satellite systems offer the only positioning technology that is
  available worldwide, for free, enabling countless applications.
  u-blox’s standard precision technology delivers meter-level positioning with products
    tailored to meet every customer’s requirements, including small size, low power
    consumption, and reliability. u-blox GNSS portfolio serves the automotive and
    industrial markets with GNSS chips and modules.
impl: implementation
IPC: Inter-process communication
ISO-TP: international standard organization transport protocol
LTA: Lane Tracing Assist
MHP: Multiple Hypotheses Prediction
MPC: model predictive control theory
np: Numpy: https://www.geeksforgeeks.org/numpy-ndarray/
ONNX: Open Neural Network Exchange
OP: openpilot
  OPAN: OP Attention Net
  OPEN: OP EfficientNet2
  OPGRU: OP GRU Recurrent Neural Net
  OPPN: OP PoseNet
  OPYL: OP Yolact
OCP: optimal control problems
OpenCL: Open Computing Language
  OpenCL: A Hands-on Introduction
  OPENCL BUFFERS AND COMPLETE EXAMPLES
  An example of OpenCL program
  OpenCL Basics
OSM: OpenStreetMap
PCM: power-train control module
PT can: Power-Train is an assembly of all components (engine, transmission,
  driveshaft, axles, and differential) that pushes your vehicle forward.
POSIX: Portable Operating System Interface
Qt for Beginners
  Qt5 Tutorial Hello World - 2020
  Qt and C++ Embedded Development
    Qt is a C++ toolkit for graphical user interface (GUI) and application development.
    Qt offers the Automotive Suite software modules and tools for building
      in-vehicle infotainment (IVI) systems.
Radar: Millimeter-wave radar has a strong ability to penetrate fog, smoke, and dust when
  compared with optical sensors such as infrared sensors, laser sensors, and cameras.
SCons User Guide 1.2.0
  SCons is a software construction tool or make tool, a software utility for
    building software (or other files) and keeping built software up-to-date
    whenever the underlying input files change.
slcan: serial line CAN
SNPE: Snapdragon Neural Processing Engine
Socket Programming in Python (Guide) "127.0.0.1"
STL: stereolithography, STL is a file format commonly used for 3D printing and
  computer-aided design (CAD).
Swag: Stuff We All Get
TCP: Transmission Control Protocol
TPL: Task Parallel Library
UDS: Unified Diagnostic Services
ULL: Unsigned Long Long, 64 bits
USB: Universal Serial Bus
VIN: Vehicle Identification Number
zmq: ZeroMQ (ØMQ, 0MQ, 0 message queue) is an asynchronous messaging system
