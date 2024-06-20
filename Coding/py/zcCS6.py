CAN 16-18.   JLL, 2021.5.11-5.25, 6.6
--------------------------------------------------

CAN 18   2021.5.11-5.25, 6.6
Do CAN 15 first in zcCS5.py
Code Overview: All files producing the fingerprint output in CAN 15.

---------- Step 1: Run get_fingerprint.py
root@localhost:/data/openpilot$ PYTHONPATH=/data/openpilot PREPAREONLY=1 /data/openpilot/selfdrive/debug/get_fingerprint.py
----- Output:
number of messages 130:
fingerprint 36: 8, 37: 8, 166: 8, ...

---------- Step 2: Code Structure Running boardd.cc
root@localhost:/$ cat comma.sh
root@localhost:/data/openpilot$ cat launch_chffrplus.sh
/home/jinn/OP081/selfdrive/manager.py
/home/jinn/OP081/selfdrive/launcher.py
/home/jinn/OP081/selfdrive/pandad.py
/home/jinn/OP081/selfdrive/boardd/boardd.cc
/home/jinn/OP081/selfdrive/boardd/panda.cc
/home/jinn/OP081/cereal/messaging/socketmaster.cc
/home/jinn/OP081/cereal/messaging/messaging.cc
/home/jinn/OP081/cereal/messaging/impl_zmq.cc
/home/jinn/OP081/selfdrive/boardd/panda.h
/home/jinn/OP081/cereal/messaging/messaging.hpp
/home/jinn/OP081/cereal/messaging/impl_zmq.hpp
/home/jinn/OP081/cereal/gen/cpp/log.capnp.h

---------- Step 3: Code Structure Running get_fingerprint.py
/home/jinn/OP081/selfdrive/debug/get_fingerprint.py
----- Input: 'can'
----- Output: "fingerprint {0}"
/home/jinn/OP081/cereal/messaging/__init__.py
/home/jinn/OP081/cereal/__init__.py
/home/jinn/OP081/cereal/log.capnp
/home/jinn/OP081/cereal/car.capnp
/home/jinn/OP081/cereal/include/c++.capnp
/home/jinn/OP081/cereal/messaging/messaging_pyx.pyx
/home/jinn/OP081/cereal/messaging/messaging.pxd
/home/jinn/OP081/cereal/messaging/messaging.hpp
/home/jinn/OP081/cereal/messaging/messaging.cc
/home/jinn/OP081/cereal/messaging/impl_zmq.hpp
/home/jinn/OP081/cereal/messaging/impl_zmq.cc
/home/jinn/OP081/cereal/services.h
--------------------------------------------------

---------- Step 1: Running Commands of get_fingerprint.py

--- /home/jinn/openpilot/selfdrive/debug/get_fingerprint.py
--- run get_fingerprint.py on comma2 installed in car
root@localhost:/data/openpilot$ PYTHONPATH=/data/openpilot PREPAREONLY=1 /data/openpilot/selfdrive/debug/get_fingerprint.py
----- Output:
number of messages 130:
fingerprint 36: 8, 37: 8, 166: 8, ...
--- PYTHONPATH is an environment variable which you can set to add additional
      directories where python will look for modules and packages

--- on PC
(OP) jinn@Liu:~/openpilot/acan$ env | grep PYTHONPATH
PYTHONPATH=/home/jinn/openpilot
(OP) jinn@Liu:~/openpilot/acan$ export PYTHONPATH=${PYTHONPATH}:${HOME}/acpp
(OP) jinn@Liu:~/openpilot/acan$ env | grep PYTHONPATH
/home/jinn/openpilot:/home/jinn/acpp
(OP) jinn@Liu:~/openpilot/acan$ PYTHONPATH=/home/jinn/openpilot
(OP) jinn@Liu:~/openpilot/acan$ env | grep PYTHONPATH
PYTHONPATH=/home/jinn/openpilot
(OP) jinn@Liu:~/openpilot/acan$ PREPAREONLY=1
--- export - set the export attribute for variables
    Exported variables add to the environment variables list and
      get passed on to child processes, not-exported variables do not.

--- run get_fingerprint.py on PC
(OP) jinn@Liu:~/openpilot/acan$ PYTHONPATH=/home/jinn/openpilot PREPAREONLY=1 /home/jinn/openpilot/selfdrive/debug/get_fingerprint.py
--- nothing from PC
--- selfdrive/manager/manager.py:
      prepare_only = os.getenv("PREPAREONLY") is not None ...
      OS module in Python provides functions for interacting with the operating system.
      os.environ returns a dictionary having user’s environmental variable
        as key and their values as value. os.environ == os.getenv
CCCCCCCCCCCCCCCCCCCCCCCC
import os
#print(os.environ)
print(os.environ.get('USER'))
print(os.environ['HOME'])
for item, value in os.environ.items():
    print('{}: {}'.format(item, value))

---------- Step 2: Code Structure Running boardd.cc

----- turn "off" the car
root@localhost:/data/openpilot$ reboot
root@localhost:/data/openpilot$ tmux a
------ now in bash 0 and running "manager.py" and "boardd.cc"

--- list all background processes
root@localhost:/data/openpilot$ ps aux root
PID   USER     TIME  COMMAND
 1457 system    0:00 {ndroid.settings} com.android.settings
 1837 root      0:00 {/data/data/com.} /data/data/com.termux/files/usr/bin/tmux new-session -s comma -d /comma.sh
 1848 root      0:00 {launch_chffrplu} /usr/bin/bash /data/openpilot/launch_chffrplus.sh
 1956 root      0:01 python3 ./manager.py
 2054 root      0:03 python -m selfdrive.athena.manage_athenad
 2055 root      0:01 selfdrive.thermald.thermald
 2056 root      0:00 selfdrive.logmessaged
 2058 root      0:08 ./_ui
 2059 root      0:00 selfdrive.loggerd.uploader
 2062 root      0:00 selfdrive.loggerd.deleter
 2067 root      0:00 ./logcatd
 2068 root      0:00 selfdrive.tombstoned
 2070 root      0:19 ./_sensord
 2216 u0_a22    0:13 {ma.plus.offroad} ai.comma.plus.offroad
 2231 root      0:01 selfdrive.pandad
 2266 root      0:02 selfdrive.athena.athenad
 3033 root      0:00 [kworker/3:1]
 3037 root      0:00 /data/data/com.termux/files/usr/bin/sshd -D -R
 3040 root      0:00 /system/comma/usr/bin/bash -l
 3069 root      0:00 ps aux root
--- tmux is a terminal multiplexer: it enables a number of terminals to be created, accessed,
      and controlled from a single screen. tmux may be detached (tmux -d) from a screen and continue
      running in the background, then later reattached (tmux a).
    http://manpages.ubuntu.com/manpages/precise/en/man1/tmux.1.html
    new-session [-d] [-n window-name] [-s session-name]
      The new session is attached to the current terminal unless -d is given.
      /comma.sh: /: root directory
--- Bash (AKA Bourne Again Shell) is a type of interpreter that processes shell
      commands in plain text format and calls Operating System services to do something.
    What is Bash? https://opensource.com/resources/what-bash
    On Linux, most commands are stored by default in system directories like /usr/bin and /bin.
    $ which bash
    https://linuxconfig.org/bash-scripting-tutorial-for-beginners
--- manager.py is the first running boardd.cc by selfdrive.pandad.

/home/jinn/OP081/comma201018.sh
  # comma.sh is not in the clone package OP081 but in comma two
root@localhost:/$ cat comma.sh
  #!/usr/bin/bash
    exec /data/data/com.termux/files/continue.sh

/home/jinn/OP081/continue201018.sh
root@localhost:/data/data/com.termux/files$ cat continue.sh
  cd /data/openpilot
  exec ./launch_openpilot.sh

/home/jinn/OP081/launch_openpilot.sh
  export PASSIVE="0"
  exec ./launch_chffrplus.sh

/home/jinn/OP081/launch_chffrplus.sh
root@localhost:/data/openpilot$ cat launch_chffrplus.sh
  # start manager
  cd selfdrive
  ./manager.py

/home/jinn/OP081/selfdrive/manager.py
  from typing import Dict, List
  from multiprocessing import Process
  if __name__ == "__main__":
  def build():
    for retry in [True, False]:
      # run scons: https://drive.google.com/file/d/1VznxlVXSCHjoE2KAUZfuYJfpnoNQQjuG/view
      env = os.environ.copy()
      env['SCONS_PROGRESS'] = "1"
      env['SCONS_CACHE'] = "1"
      nproc = os.cpu_count()
      j_flag = "" if nproc is None else f"-j{nproc - 1}"
      scons = subprocess.Popen(["scons", j_flag], cwd=BASEDIR, env=env, stderr=subprocess.PIPE)
  if __name__ == "__main__" and not PREBUILT:
    build()
  from selfdrive.launcher import launcher
  running: Dict[str, Process] = {}
  managed_processes = {
    "pandad": "selfdrive.pandad",   # file: pandad.py
    "boardd": ("selfdrive/boardd", ["./boardd"]),   # not used directly
  def start_managed_process(name):
    proc = managed_processes[name]
    if isinstance(proc, str):
      cloudlog.info("starting python %s" % proc)
--- name == "pandad"
      running[name] = Process(name=name, target=launcher, args=(proc,))
    else:
      pdir, pargs = proc
      cwd = os.path.join(BASEDIR, pdir)
      cloudlog.info("starting process %s" % name)
      running[name] = Process(name=name, target=nativelauncher, args=(pargs, cwd))
    running[name].start()
  def manager_thread():
    cloudlog.info("manager start")
    if os.getenv("NOBOARD") is None:
      start_managed_process("pandad")
  def main():
    try:
      manager_thread()
  if __name__ == "__main__":
    try:
      main()
--- my_env = os.environ.copy(): Copy all env variables
    my_env["PATH"] = "/usr/sbin:/sbin:" + my_env["PATH"]
--- f"-j{nproc - 1}": f-strings in Python
      import datetime
      today = datetime.datetime.today()
      print(f"{today:%B %d, %Y}")
      Output: April 04, 2018
      https://docs.python.org/3/library/datetime.html
--- subprocess.Popen(my_command, env=my_env)
    https://docs.python.org/3/library/subprocess.html
    The subprocess module defines one class, Popen and a few wrapper functions
      that use that class. The constructor for Popen takes arguments to set up
      the new process so the parent can communicate with it via PIPE.
--- running: Dict[str, Process] = {}
    This is how you declare the type of a variable in Python 3.6
      age: int = 1
    In Python 3.5 and earlier:
      age = 1  # type: int
--- isinstance(proc, str): Check if the number 5 is an integer: x = isinstance(5, int)
    "selfdrive.pandad" == str
--- https://docs.python.org/3/library/multiprocessing.html
    class multiprocessing.Process(group=None, target=None, name=None, args=(), kwargs={}, *, daemon=None)
    multiprocessing, Process-based parallelism, is a package that supports
      spawning processes using an API similar to the threading module.
    In multiprocessing, processes are spawned by creating a Process object and
      then calling its start() method.
    https://pymotw.com/2/multiprocessing/basics.html
--- Python uses the keyword None to define null objects and variables.
      None is not the same as 0, False, or an empty string.
--- The try block lets you test a block of code for errors.
    The except block lets you handle the error.
    The finally block lets you execute code, regardless of the result of the try- and except blocks.

/home/jinn/OP081/selfdrive/launcher.py
  import importlib
  def launcher(proc):
    try:
      mod = importlib.import_module(proc) # import the process
      mod.main() # exec the process

/home/jinn/OP081/selfdrive/pandad.py
  from panda import BASEDIR, Panda, PandaDFU, build_st
  def main():
    os.chdir("boardd")
    os.execvp("./boardd", ["./boardd"])
  if __name__ == "__main__":
    main()
--- Both chdir and cd (Unix command) stand for "change directory".
    https://note.nkmk.me/en/python-os-getcwd-chdir/
--- os.execvp(file, args) executes a new program file replacing the current process and does not return.
    The “v” means that the number of parameters is variable,
      with the arguments being passed in a list or tuple as the args parameter.
    The “p” uses the PATH environment variable to locate the program file.
    https://stackless.readthedocs.io/en/2.7-slp/library/os.html

/home/jinn/OP081/selfdrive/boardd/boardd.cc
  #include "cereal/gen/cpp/car.capnp.h"
  #include "messaging.hpp" ??? "cereal/messaging/messaging.h"
  #include "panda.h"
  #define NIBBLE_TO_HEX(n) ((n) < 10 ? (n) + '0' : ((n) - 10) + 'a')
  Panda * panda = NULL;
  void can_recv(PubMaster &pm) {
    MessageBuilder msg; // create message
    auto event = msg.initEvent(); // event == msg
    panda->can_receive(event);
----- Output: msg in pm.send("can", msg);
  void can_recv_thread() {
    PubMaster pm({"can"});  // can = 8006
    const uint64_t dt = 10000000ULL; // run at 100hz
    uint64_t next_frame_time = nanos_since_boot() + dt;
    while (!do_exit && panda->connected) {
      can_recv(pm);
  int main() {
    while (!do_exit){
      std::vector<std::thread> threads;
----- Output
      threads.push_back(std::thread(can_recv_thread));
      for (auto &t : threads) t.join();
--- https://www.geeksforgeeks.org/multithreading-in-cpp/
    The class thread represents a single thread of execution.
    Threads allow multiple functions to execute concurrently.
    std::thread is the thread class that represents a single thread in C++.
    std::thread(callable)
    A callable can be either of the three: A function pointer; A function object; A lambda expression
    ULL (Unsigned Long Long, 64 bits)

/home/jinn/OP081/selfdrive/boardd/panda.h
  #include "cereal/gen/cpp/car.capnp.h"
  #include "cereal/gen/cpp/log.capnp.h"
  class Panda {
    private:
      libusb_device_handle *dev_handle = NULL;
    public:
      Panda();
      int usb_bulk_read(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT);
      int can_receive(cereal::Event::Builder &event);

/home/jinn/OP081/selfdrive/boardd/panda.cc
  #include "panda.h"
  int Panda::usb_bulk_read(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout) {
    int transferred = 0;
    do {
      err = libusb_bulk_transfer(dev_handle, endpoint, data, length, &transferred, timeout);
      if (err == LIBUSB_ERROR_TIMEOUT) {
        break; // timeout is okay to exit, recv still happened
      } else if (err == LIBUSB_ERROR_OVERFLOW) {
        LOGE_100("overflow got 0x%x", transferred);
      } else if (err != 0) {
        handle_usb_issue(err, __func__);}
    } while(err != 0 && connected);
    return transferred;
    void Panda::can_send(capnp::List<cereal::CanData>::Reader can_data_list){
--- panda210516.cc
  int Panda::can_receive(kj::Array<capnp::word>& out_buf) {
  int Panda::can_receive(cereal::Event::Builder &event){
    uint32_t data[RECV_SIZE/4];
    int recv = usb_bulk_read(0x81, (unsigned char*)data, RECV_SIZE);
    size_t num_msg = recv / 0x10;
----- Output: canData == event
    auto canData = event.initCan(num_msg);
      canData[i].setAddress(data[i*4] >> 3);
      canData[i].setAddress(data[i*4] >> 21);
      canData[i].setBusTime(data[i*4+1] >> 16);
    int len = data[i*4+1]&0xF;
    canData[i].setDat(kj::arrayPtr((uint8_t*)&data[i*4+2], len));
    canData[i].setSrc((data[i*4+1] >> 4) & 0xff);
  return recv;
--- usb_bulk_read: USB-C cables can be used to quickly charge many popular devices
      and transfer data faster than any other USB type.
    https://comma.ai/setup

/home/jinn/OP081/cereal/messaging/messaging.hpp
  #include <capnp/serialize.h>
  #include "../gen/cpp/log.capnp.h"
  class PubSocket {
  public:
    virtual int connect(Context *context, std::string endpoint) = 0;
    virtual int send(char *data, size_t size) = 0; # defined in impl_zmq.cc
    static PubSocket * create();
    static PubSocket * create(Context * context, std::string endpoint);
  class MessageBuilder : public capnp::MallocMessageBuilder {
  public:
    MessageBuilder() = default;
    cereal::Event::Builder initEvent(bool valid = true) {
      cereal::Event::Builder event = initRoot<cereal::Event>();
      event.setLogMonoTime(current_time);
      return event;
  class PubMaster {
  public:
    PubMaster(const std::initializer_list<const char *> &service_list);
    inline int send(const char *name, capnp::byte *data, size_t size)
      { return sockets_.at(name)->send((char *)data, size); }
----- Output: msg
    int send(const char *name, MessageBuilder &msg); # defined in socketmaster.cc
  private:
    std::map<std::string, PubSocket *> sockets_;
--- MessageBuilder: cereal::Event::Builder
    To create a new message, most users use capnp::MallocMessageBuilder.
      First allocate a MessageBuilder buffer: message for your message to go in.
      Then allocate an actual object: addressBook from your schema <AddressBook>.
        ::capnp::MallocMessageBuilder message;
        AddressBook::Builder addressBook = message.initRoot<AddressBook>();
      https://capnproto.org/cxx.html
--- std::map<K,V> is an associative container that store elements in key-value pair.
    std::map::at returns a reference to the mapped value of the element identified with key k.
      std::map<std::string,int> mymap = {{ "alpha", 0 }, { "beta", 0 } };
      mymap.at("alpha") = 10;

/home/jinn/OP081/cereal/messaging/socketmaster.cc
  #include "messaging.hpp"
  int PubMaster::send(const char *name, MessageBuilder &msg) {
    auto bytes = msg.toBytes();
----- Output: msg
    return send(name, bytes.begin(), bytes.size());

/home/jinn/OP081/cereal/messaging/messaging.cc
  #include "messaging.hpp"
  #include "impl_zmq.hpp"
  PubSocket * PubSocket::create(){
    PubSocket * s;
    if (std::getenv("ZMQ") || MUST_USE_ZMQ){
      s = new ZMQPubSocket();
    return s;
  PubSocket * PubSocket::create(Context * context, std::string endpoint){
    PubSocket *s = PubSocket::create();
    int r = s->connect(context, endpoint);
      return s;

/home/jinn/OP081/cereal/messaging/impl_zmq.hpp
  class ZMQPubSocket : public PubSocket {
    int connect(Context *context, std::string endpoint);
    int send(char *data, size_t size);

/home/jinn/OP081/cereal/messaging/impl_zmq.cc
  int ZMQPubSocket::connect(Context *context, std::string endpoint){
    sock = zmq_socket(context->getRawContext(), ZMQ_PUB);
    full_endpoint = "tcp://*:";
    full_endpoint += std::to_string(get_port(endpoint));
    return zmq_bind(sock, full_endpoint.c_str());
  int ZMQPubSocket::send(char *data, size_t size){
    return zmq_send(sock, data, size, ZMQ_DONTWAIT);

/home/jinn/OP081/cereal/gen/cpp/log.capnp.h
  // Generated by Cap'n Proto compiler, DO NOT EDIT
  // source: log.capnp
  #include "car.capnp.h"
  namespace capnp {
  namespace schemas {
  class Event::Builder {
    inline void setLogMonoTime( ::uint64_t value);

---------- Step 3: Code Structure Running get_fingerprint.py

----- now in bash 1
root@localhost:/data/openpilot$ PYTHONPATH=/data/openpilot PREPAREONLY=1 /data/openpilot/selfdrive/debug/get_fingerprint.py
--- turn on the car
----- Output:
number of messages 130:
fingerprint 36: 8, 37: 8, ...

/home/jinn/OP081/selfdrive/debug/get_fingerprint.py
  import cereal.messaging as messaging
----- Input: 'can'
  logcan = messaging.sub_sock('can')
  msgs = {}
    lc = messaging.recv_sock(logcan, True)
    for c in lc.can:
      # read also msgs sent by EON on CAN bus 0x80 and filter out the
      # addr with more than 11 bits
      if c.src in [0, 2] and c.address < 0x800:
        msgs[c.address] = len(c.dat)
  fingerprint = ', '.join("%d: %d" % v for v in sorted(msgs.items()))
----- Output: "fingerprint {0}"
  print("number of messages {0}:".format(len(msgs)))
  print("fingerprint {0}".format(fingerprint))

/home/jinn/OP081/cereal/messaging/__init__.py
  from .messaging_pyx import Context, Poller, SubSocket
  import capnp
  from typing import Optional, List, Union
  from cereal import log
  def sub_sock(endpoint: str, poller: Optional[Poller] = None, addr: str = "127.0.0.1",
    conflate: bool = False, timeout: Optional[int] = None) -> SubSocket:
    sock = SubSocket()
----- Input: endpoint == 'can', addr == "127.0.0.1"
    sock.connect(context, endpoint, addr.encode('utf8'), conflate)
  return sock
  def recv_sock(sock: SubSocket, wait: bool = False) -> Union[None, capnp.lib.capnp._DynamicStructReader]:
----- Output:
    rcv = sock.receive()
    dat = rcv
----- Output: Event: can @5 :List(CanData)
    dat = log.Event.from_bytes(dat)
  return dat
--- The name localhost normally resolves to the IPv4 loopback address 127.0.0.1.

/home/jinn/OP081/cereal/__init__.py
  import capnp
  log = capnp.load(os.path.join(CEREAL_PATH, "log.capnp"))
  car = capnp.load(os.path.join(CEREAL_PATH, "car.capnp"))

/home/jinn/OP081/cereal/log.capnp
  using Cxx = import "./include/c++.capnp";
  $Cxx.namespace("cereal");
  using Car = import "car.capnp";
  struct CanData {
    address @0 :UInt32;
    busTime @1 :UInt16;
    dat     @2 :Data;
    src     @3 :UInt8;
  struct RadarState @0x9a185389d6fdd05f {
  struct LiveTracks {
    dRel @1 :Float32;
  struct ControlsState @0x97ff69c53601abf1 {
  struct ModelData {
  struct ModelSettings {
  struct MetaData {
  struct LongitudinalData {
    speeds @0 :List(Float32);
  struct EncodeIndex {
    # picture from camera
    frameId @0 :UInt32;
  struct Plan {
    # lateral, 3rd order polynomial
    enum LongitudinalPlanSource {
  struct PathPlan {
  struct LiveLocationKalman {
  struct LiveLocationData {
  struct Event {
    logMonoTime @0 :UInt64;  # in nanoseconds?
    union {
      frame @2 :FrameData;
      can @5 :List(CanData);
      controlsState @7 :ControlsState;
      model @9 :ModelData;
      sendcan @17 :List(CanData);
      carState @22 :Car.CarState;
      carParams @69: Car.CarParams;
      carEvents @68: List(Car.CarEvent);
      carParams @69: Car.CarParams;

/home/jinn/OP081/cereal/car.capnp
  struct CarState {
  struct Actuators {
  struct CarParams {
    struct LongitudinalPIDTuning {
    struct LateralINDITuning {
    enum SteerControlType {
    struct CarFw {
    enum Ecu {
    enum FingerprintSource {
      can @0;
      fw @1;

/home/jinn/OP081/cereal/include/c++.capnp
  $namespace("capnp::annotations");

/home/jinn/OP081/cereal/messaging/messaging_pyx.pyx
  from messaging cimport SubSocket as cppSubSocket   # from messaging.pxd
  cdef class SubSocket:
    cdef cppSubSocket * socket
    def __cinit__(self):
      self.socket = cppSubSocket.create()
----- Input:  endpoint == 'can', address == "127.0.0.1"
    def connect(self, Context context, string endpoint, string address=b"127.0.0.1", bool conflate=False):
      r = self.socket.connect(context.context, endpoint, address, conflate)
    def receive(self, bool non_blocking=False):
----- Output:
      msg = self.socket.receive(non_blocking)
      sz = msg.getSize()
----- Output:
      m = msg.getData()[:sz]
      return m

/home/jinn/OP081/cereal/messaging/messaging.pxd
  cdef extern from "messaging.hpp":
    cdef cppclass Context:
    cdef cppclass Message:
    cdef cppclass SubSocket:
      SubSocket * create()
----- Input: string == endpoint == 'can'
      int connect(Context *, string, string, bool)
----- Output:
      Message * receive(bool)
--- cdef extern from tells Cython that the declarations are in a C header file

/home/jinn/OP081/cereal/messaging/messaging.hpp
  #include <capnp/serialize.h>
  #include "../gen/cpp/log.capnp.h"
  class Context {
  class Message {
    virtual size_t getSize() = 0; # defined in impl_zmq.hpp
    virtual char * getData() = 0; # defined in impl_zmq.hpp
  class SubSocket {
----- Input: endpoint == 'can'
    virtual int connect(Context *context, std::string endpoint, std::string address, bool conflate=false) = 0;
----- Output:
    virtual Message *receive(bool non_blocking=false) = 0; # defined in impl_zmq.cc
    virtual void * getRawSocket() = 0; # defined in impl_zmq.hpp
    static SubSocket * create();
--- A virtual function is a member function which is declared within
    a base class and is re-defined (overriden) by a derived class.
    https://www.geeksforgeeks.org/virtual-function-cpp/

/home/jinn/OP081/cereal/messaging/messaging.cc
  #include "messaging.hpp"
  #include "impl_zmq.h"
  SubSocket * SubSocket::create(){
    SubSocket * s;
      s = new ZMQSubSocket();
    return s;}

/home/jinn/OP081/cereal/messaging/impl_zmq.hpp
  class ZMQMessage : public Message {
    size_t getSize(){return size;}
    char * getData(){return data;}
  class ZMQSubSocket : public SubSocket {
----- Input: endpoint == 'can'
    int connect(Context *context, std::string endpoint, std::string address, bool conflate=false);
    void * getRawSocket() {return sock;}
    Message *receive(bool non_blocking=false);

/home/jinn/OP081/cereal/messaging/impl_zmq.cc
  #include <zmq.h>
  #include "services.h"
  #include "impl_zmq.hpp"
----- Input: endpoint == 'can'
  static int get_port(std::string endpoint) {
    for (const auto& it : services) {
      std::string name = it.name;
      if (name == endpoint) {
----- Input: .name = "can", .port = 8006 in services.h
        port = it.port;
        break;
    return port;
  ZMQContext::ZMQContext() {
    context = zmq_ctx_new();
  int ZMQSubSocket::connect(Context *context, std::string endpoint, std::string address, bool conflate, bool check_endpoint){
    sock = zmq_socket(context->getRawContext(), ZMQ_SUB);
    full_endpoint = "tcp://" + address + ":";
    return zmq_connect(sock, full_endpoint.c_str());
  Message * ZMQSubSocket::receive(bool non_blocking){
    zmq_msg_t msg;
----- Output:
    int rc = zmq_msg_recv(&msg, sock, flags);
    Message *r = NULL;
    if (rc >= 0){
      // Make a copy to ensure the data is aligned
      r = new ZMQMessage;
      r->init((char*)zmq_msg_data(&msg), zmq_msg_size(&msg));
    zmq_msg_close(&msg);
    return r;
--- How To Work with the ZeroMQ Messaging Library
    ZMQ: ZeroMQ (ØMQ, 0MQ, 0 message queue) is an asynchronous messaging system
    toolkit (i.e. a library) between applications and processes - fast and asynchronously.
    tcp (Transmission Control Protocol) impl (implementation)
    In computer networking, a port is a communication endpoint.
    A socket is externally identified to other hosts by its socket address, which is
    the triad of transport protocol, IP address, and port number. Example:
      Socket mysocket = getSocket(type = "TCP")
      connect(mysocket, address = "1.2.3.4", port = "80")
      send(mysocket, "Hello, world!")
      close(mysocket)
    Define the socket using the "Context"

/home/jinn/OP081/cereal/services.h
  struct service { char name[0x100]; int port; bool should_log; int frequency; int decimation; };
  static struct service services[] = {
    { .name = "frame", .port = 8002, .should_log = true, .frequency = 20, .decimation = 1 },
    { .name = "can", .port = 8006, .should_log = true, .frequency = 100, .decimation = -1 },

--------------------------------------------------
CCCCCCCCCCCCCCCCCCCCCCCC
'''
CAN 17   2021.5.9
??? result():
openpilot/selfdrive/car/fingerprints.py

openpilot/selfdrive/debug/show_matching_cars.py
'''
def get_attr_from_cars(attr, result=dict, combine_brands=True):
  result = result()
  print(' attr = ', attr)
  print(' result = ', result)
  #print(' result() = ', result())
  print(' dict = ', dict)
  print(' combine_brands = ', combine_brands)
FW_VERSIONS = get_attr_from_cars('FW_VERSIONS')
# Python dict() Function
#x = dict(name = "John", age = 36, country = "Norway")
#print(x)

--------------------------------------------------
CCCCCCCCCCCCCCCCCCCCCCCC
'''
CAN 16    2021.5.8
??? ->: def new_message(service: Optional[str] = None, size: Optional[int] = None)
          -> capnp.lib.capnp._DynamicStructBuilder: ...     in
cereal/messaging/__init__.py     called by
openpilot/selfdrive/debug/get_fingerprint.py
  CAN 15    ---------- Step 3: Get Fingerprint

--- What does -> mean in Python function definitions?
    It's a function annotation.

def foo(a: 'x', b: 5 + 6, c: list) -> max(2, 9):
would result in an __annotations__ mapping of
{'a': 'x',
 'b': 11,
 'c': list,
 'return': 9}

def f(x) -> int:
    return int(x)
the -> int just tells that f() returns an integer (but it doesn't
force the function to return an integer). It is called a return
annotation, and can be accessed as f.__annotations__['return'].
'''
def kinetic_energy(m:'in KG', v:'in M/S') -> 'Joules': return 1/2*m*v**2
kinetic_energy.__annotations__
{'return': 'Joules', 'v': 'in M/S', 'm': 'in KG'}
# returns a string
print("output: {:,} {}".format(kinetic_energy(20, 3000),
      kinetic_energy.__annotations__['return']))
# output: 90,000,000.0 Joules
rd = {'type':float, 'units':'Joules',
      'docstring':'Given mass and velocity returns kinetic energy in Joules'}
def f() -> rd: pass
# returns a data (class) structure
print(f.__annotations__['return'])
print(f.__annotations__['return']['type'])
print(f.__annotations__['return']['units'])
print(f.__annotations__['return']['docstring'])
