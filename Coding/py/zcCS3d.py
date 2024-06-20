CAN 10d   JLL, 2021.4.21-4.27, 6.20-6.22, 6.30-7.1
Code Overview: From Neural Net to CAN by Steering
--------------------------------------------------

---------- Step 3: Controls ----- Messaging
----- Output 1: pm.send.pub_sock.pyx.pxd.hpp.PubSocket.cc.ZMQPubSocket.zmq_send
----- Return: Input 1: sub_sock.pyx.pxd.hpp.SubSocket.cc.ZMQSubSocket.zmq_connect
----- Return: Output 1: pub_sock.pyx.pxd.hpp.PubSocket.cc.ZMQPubSocket.zmq_bind
----- Keywords: Socket

---------- Step 4: Board
----- Input: can_recv_thread.can_recv.panda->can_receive
----- Output 1: pm.send: can_send_thread.SubSocket.ZMQSubSocket.receive.zmq_msg_recv.
                  Panda.can_send.memcpy(&,,).usb_bulk_write(3, data, size, 5)
----- Keywords: can_send, panda

/OP081/selfdrive/boardd/boardd.cc
/OP081/cereal/messaging/messaging.hpp
/OP081/cereal/messaging/messaging.cc
/OP081/cereal/messaging/impl_zmq.hpp
/OP081/cereal/messaging/impl_zmq.cc
/OP081/selfdrive/boardd/panda.h
/OP081/selfdrive/boardd/panda.cc
--------------------------------------------------

----- Messaging

/OP081/cereal/messaging/__init__.py
  # must be build with scons
  from .messaging_pyx import Context, Poller, SubSocket, PubSocket
  import capnp
  from typing import Optional, List, Union
  from cereal import log
  from cereal.services import service_list
  context = Context()
----- Return: Output 2: messaging.new_message.log(capnp).new_message
  def new_message(service: Optional[str] = None, size: Optional[int] = None) -> capnp.lib.capnp._DynamicStructBuilder:
    dat = log.Event.new_message()
    if service is not None:
      if size is None:
        dat.init(service)
      else:
        dat.init(service, size)
    return dat
----- Return: Output 1: pub_sock.PubSocket.connect
  def pub_sock(endpoint: str) -> PubSocket:
    sock = PubSocket()
    sock.connect(context, endpoint)
    return sock
----- Return: Input 1: sub_sock.SubSocket.connect(endpoint=='can')
  def sub_sock(endpoint: str, , , , timeout) -> SubSocket:
    sock = SubSocket()
    sock.connect(context, endpoint, addr.encode('utf8'), conflate)
  return sock
  class SubMaster():
    def __init__(self, services: List[str], , , ):
      self.sock = {}
      self.data = {}
      for s in services:
          self.sock[s] = sub_sock(s, poller=p, addr=addr, conflate=True)
        try:
          data = new_message(s)
        self.data[s] = getattr(data, s)
----- Output 1: pm.send.capnp.to_bytes.pub_sock.send
  class PubMaster():
    def __init__(self, services: List[str]):
      self.sock = {}
      for s in services:
        self.sock[s] = pub_sock(s)
    def send(self, s: str, dat: Union[bytes, capnp.lib.capnp._DynamicStructBuilder]) -> None:
      if not isinstance(dat, bytes):
        dat = dat.to_bytes()
      self.sock[s].send(dat)

--- log.Event.new_message() => capnp.new_message(): see zcCS3c
--- dat.to_bytes() => capnp.to_bytes(): https://jparyani.github.io/pycapnp/quickstart.html

/OP081/cereal/__init__.py
  import capnp
  log = capnp.load(os.path.join(CEREAL_PATH, "log.capnp"))

/OP081/cereal/log.capnp
--- see zcCS3a
  using Cxx = import "./include/c++.capnp";
  $Cxx.namespace("cereal");
  struct ControlsState @0x97ff69c53601abf1 {
    state @31 :OpenpilotState;
    vEgo @0 :Float32;
    longControlState @30 :LongControlState;
    angleSteers @13 :Float32;     # Steering angle in degrees.
    angleSteersDes @29 :Float32;
    curvature @37 :Float32;       # path curvature from vehicle model
    lateralControlState :union {
      indiState @52 :LateralINDIState;
    enum OpenpilotState @0xdbe58b96d2d1ac61 {
      disabled @0;
      enabled @2;
    struct LateralINDIState {
      steerAngle @1 :Float32;
      output @9 :Float32;
  struct PathPlan {
    laneWidth @0 :Float32;
    dPoly @1 :List(Float32);
    cPoly @2 :List(Float32);
    lPoly @4 :List(Float32);
    rPoly @6 :List(Float32);
    angleSteers @8 :Float32; # deg
    rateSteers @13 :Float32; # deg/s
    angleOffset @11 :Float32;
  struct Event {
    logMonoTime @0 :UInt64;  # in nanoseconds?
    union {
      controlsState @7 :ControlsState;
      carState @22 :Car.CarState;
      carControl @23 :Car.CarControl;

/OP081/cereal/include/c++.capnp
  $namespace("capnp::annotations");

/OP081/cereal/messaging/messaging_pyx.pyx
  from .messaging cimport Context as cppContext
  from .messaging cimport SubSocket as cppSubSocket   # from messaging.pxd
  from .messaging cimport PubSocket as cppPubSocket
  cdef class Context:
    cdef cppContext * context
    def __cinit__(self):
      self.context = cppContext.create()
----- Return: Input 1: sub_sock.pyx.SubSocket.cppSubSocket.connect
  cdef class SubSocket:
    cdef cppSubSocket * socket
    def __cinit__(self):
      self.socket = cppSubSocket.create()
    def connect(self, Context context, string endpoint, string address=b"127.0.0.1", bool conflate=False):
      r = self.socket.connect(context.context, endpoint, address, conflate)
    def receive(self, bool non_blocking=False):
      msg = self.socket.receive(non_blocking)
      sz = msg.getSize()
      m = msg.getData()[:sz]
      return m
----- Return: Output 1: pub_sock.pyx.PubSocket.cppPubSocket.connect
----- Output 1: pm.send.pub_sock.pyx.PubSocket.cppPubSocket.send
  cdef class PubSocket:
    cdef cppPubSocket * socket
    def __cinit__(self):
      self.socket = cppPubSocket.create()
    def connect(self, Context context, string endpoint):
      r = self.socket.connect(context.context, endpoint)
    def send(self, bytes data):
      length = len(data)
      r = self.socket.send(<char*>data, length)

--- https://pythonprogramming.net/introduction-and-basics-cython-tutorial/
    https://riptutorial.com/cython
    Cython is Python with C data types. pyx is Cython file
    Cython is an intermediary between Python and C/C++.
    Numpy, Pandas, and Scikit-learn all make use of Cython!
    The main crux: Add typing information (Python: dynamic typing vs Cython: static typing)
    .pxd files work like C header files and contain Cython declarations
    a pxd file is imported into a pyx module using cimport
--- endpoint == 'can', 'sendcan', etc.

/OP081/cereal/messaging/messaging.pxd
  cdef extern from "messaging.hpp":
    cdef cppclass Context:
    cdef cppclass Message:
----- Return: Input 1: sub_sock.pyx.pxd.SubSocket.connect
    cdef cppclass SubSocket:
      SubSocket * create()
      int connect(Context *, string, string, bool)
      Message * receive(bool)
----- Return: Output 1: pub_sock.pyx.pxd.PubSocket.connect
----- Output 1: pm.send.pub_sock.pyx.pxd.PubSocket.send
    cdef cppclass PubSocket:
      @staticmethod
      PubSocket * create()
      int connect(Context *, string)
      int sendMessage(Message *)
      int send(char *, size_t)

--- string == endpoint == 'can', 'sendcan', etc.
--- cdef extern from tells Cython that the declarations are in a C header file

/OP081/cereal/messaging/messaging.hpp
  #include <capnp/serialize.h>
  #include "../gen/cpp/log.capnp.h"
  class Context {
  class Message {
    virtual size_t getSize() = 0; # defined in impl_zmq.hpp
    virtual char * getData() = 0; # defined in impl_zmq.hpp
----- Return: Input 1: sub_sock.pyx.pxd.hpp.SubSocket.connect
  class SubSocket {
    virtual int connect(Context *context, std::string endpoint, std::string address, bool conflate=false) = 0;
    virtual Message *receive(bool non_blocking=false) = 0; # defined in impl_zmq.cc
    virtual void * getRawSocket() = 0; # defined in impl_zmq.hpp
    static SubSocket * create();
----- Return: Output 1: pub_sock.pyx.pxd.hpp.PubSocket.connect
----- Output 1: pm.send.pub_sock.pyx.pxd.hpp.PubSocket.send
  class PubSocket {
    virtual int connect(Context *context, std::string endpoint) = 0;
    virtual int sendMessage(Message *message) = 0;
    virtual int send(char *data, size_t size) = 0;
    static PubSocket * create();

/OP081/cereal/messaging/messaging.cc
  #include "messaging.hpp"
  #include "impl_zmq.h"
  SubSocket * SubSocket::create(){
    SubSocket * s;
      s = new ZMQSubSocket();
    return s;}
  PubSocket * PubSocket::create(){
    PubSocket * s;
      s = new ZMQPubSocket();
    return s;}

/OP081/cereal/messaging/impl_zmq.hpp
  class ZMQMessage : public Message {
    size_t getSize(){return size;}
    char * getData(){return data;}
  class ZMQSubSocket : public SubSocket {
    int connect(Context *context, std::string endpoint, std::string address, bool conflate=false);
    void * getRawSocket() {return sock;}
    Message *receive(bool non_blocking=false);

--- Input: endpoint == 'can'

/OP081/cereal/messaging/impl_zmq.cc
  #include <zmq.h>
  #include "services.h"
  #include "impl_zmq.hpp"
  static int get_port(std::string endpoint) {
    for (const auto& it : services) {
      std::string name = it.name;
      if (name == endpoint) { port = it.port; break;
    return port;
  ZMQContext::ZMQContext() { context = zmq_ctx_new();
----- Return: Input 1: sub_sock.pyx.pxd.hpp.SubSocket.cc.ZMQSubSocket.zmq_connect
  int ZMQSubSocket::connect(Context *context, std::string endpoint, std::string address, bool conflate, bool check_endpoint){
    sock = zmq_socket(context->getRawContext(), ZMQ_SUB);
    full_endpoint = "tcp://" + address + ":";
    full_endpoint += std::to_string(get_port(endpoint));
    return zmq_connect(sock, full_endpoint.c_str());
  Message * ZMQSubSocket::receive(bool non_blocking){
    zmq_msg_t msg;
    int rc = zmq_msg_recv(&msg, sock, flags);
    Message *r = NULL;
    if (rc >= 0){
      // Make a copy to ensure the data is aligned
      r = new ZMQMessage;
      r->init((char*)zmq_msg_data(&msg), zmq_msg_size(&msg));
    zmq_msg_close(&msg);
    return r;
----- Return: Output 1: pub_sock.pyx.pxd.hpp.PubSocket.cc.ZMQPubSocket.zmq_bind
  int ZMQPubSocket::connect(Context *context, std::string endpoint){
    sock = zmq_socket(context->getRawContext(), ZMQ_PUB);
    full_endpoint = "tcp://*:";
    full_endpoint += std::to_string(get_port(endpoint));
    return zmq_bind(sock, full_endpoint.c_str());
----- Output 1: pm.send.pub_sock.pyx.pxd.hpp.PubSocket.cc.ZMQPubSocket.zmq_send
  int ZMQPubSocket::send(char *data, size_t size){
    return zmq_send(sock, data, size, ZMQ_DONTWAIT);

/OP081/cereal/services.h
--- see zcCS3a

---------- Step 4: Board

/OP081/selfdrive/boardd/boardd.cc
  #include "cereal/gen/cpp/car.capnp.h"
  #include "messaging.hpp" ??? "cereal/messaging/messaging.h"
  #include "panda.h"
  #define NIBBLE_TO_HEX(n) ((n) < 10 ? (n) + '0' : ((n) - 10) + 'a')
  Panda * panda = NULL;
  void can_recv(PubMaster &pm) {
    MessageBuilder msg; // create message
    auto event = msg.initEvent(); // event == msg
    panda->can_receive(event);
  void can_recv_thread() {
    PubMaster pm({"can"});  // can = 8006
    const uint64_t dt = 10000000ULL; // run at 100hz
    uint64_t next_frame_time = nanos_since_boot() + dt;
    while (!do_exit && panda->connected) {
      can_recv(pm);
----- Output 1: pm.send('sendcan', canData): can_send_thread.SubSocket->receive().panda->can_send()
  void can_send_thread() {
    Context * context = Context::create();
    SubSocket * subscriber = SubSocket::create(context, "sendcan");
    subscriber->setTimeout(100);
    while (!do_exit && panda->connected) {  // run as fast as messages come in
      Message * msg = subscriber->receive();
      auto amsg = kj::heapArray<capnp::word>((msg->getSize() / sizeof(capnp::word)) + 1);
      memcpy(amsg.begin(), msg->getData(), msg->getSize());
      capnp::FlatArrayMessageReader cmsg(amsg);
      cereal::Event::Reader event = cmsg.getRoot<cereal::Event>();
      //Dont send if older than 1 second
      if (nanos_since_boot() - event.getLogMonoTime() < 1e9) {
        if (!fake_send){
          panda->can_send(event.getSendcan());
  int main() {
    while (!do_exit){
      std::vector<std::thread> threads;
      usb_retry_connect();  // connect to the board
      threads.push_back(std::thread(can_send_thread));
      threads.push_back(std::thread(can_recv_thread));
      for (auto &t : threads) t.join();

--- NIBBLE_TO_HEX(n) converts a nibble (4 bits) to its uppercase hexadecimal character representation [0-9, A-F]
    The term 'nibble' originates from its representing 'half a byte', with 'byte' a homophone of the English word 'bite'.
    An 8-bit byte is split in half and each nibble is used to store one decimal digit.
    Nibble is half a byte (0-15, or one hex digit). Low nibble are the bits 0-3; high nibble are bits 4-7.
    https://referencesource.microsoft.com/#system.web/Security/Cryptography/CryptoUtil.cs,f195b3fd2b6594b3,references
--- can_recv(PubMaster &pm): boardd receives can data from panda and then publishes (pm) it
      can_recv_thread.can_recv.panda->can_receive
      can_send_thread.SubSocket->receive.panda->can_send
    boardd receives sendcan data from SubSocket (subscriber->receive()) and then sends it to CAN via panda
--- https://www.geeksforgeeks.org/multithreading-in-cpp/
    The class thread represents a single thread of execution.
    Threads allow multiple functions to execute concurrently.
    std::thread is the thread class that represents a single thread in C++.
    std::thread(callable)
    A callable can be either of the three: A function pointer; A function object; A lambda expression
    ULL (Unsigned Long Long, 64 bits)

/OP081/cereal/messaging/messaging.hpp
  class SubSocket {
----- Output 1: pm.send: can_send_thread.SubSocket.receive
    virtual Message *receive(bool non_blocking=false) = 0; # defined in impl_zmq.cc
  class PubMaster {
    PubMaster(const std::initializer_list<const char *> &service_list);

/OP081/cereal/messaging/messaging.cc
  SubSocket * SubSocket::create(){
      s = new ZMQSubSocket();

/OP081/cereal/messaging/impl_zmq.hpp
  class ZMQSubSocket : public SubSocket {

/OP081/cereal/messaging/impl_zmq.cc
----- Output 1: pm.send: can_send_thread.SubSocket.ZMQSubSocket.receive.zmq_msg_recv
  Message * ZMQSubSocket::receive(bool non_blocking){
    zmq_msg_t msg;
    int rc = zmq_msg_recv(&msg, sock, flags);
    Message *r = NULL;
    if (rc >= 0){
      // Make a copy to ensure the data is aligned
      r = new ZMQMessage;
      r->init((char*)zmq_msg_data(&msg), zmq_msg_size(&msg));
    zmq_msg_close(&msg);
    return r;

/OP081/selfdrive/boardd/panda.h
  #include <libusb-1.0/libusb.h>
  #include "cereal/gen/cpp/car.capnp.h"
  #include "cereal/gen/cpp/log.capnp.h"
  #define RECV_SIZE (0x1000) // double the FIFO size
  #define TIMEOUT 0
  struct __attribute__((packed)) health_t { // copied from panda/board/main.c
    uint32_t uptime;
    uint32_t voltage;
    uint8_t controls_allowed;
    uint8_t gas_interceptor_detected;
  class Panda {
    Panda();
    // HW communication
    int usb_bulk_write(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT);
    int usb_bulk_read(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT);
    // Panda functionality
    cereal::HealthData::HwType get_hw_type();
    const char* get_firmware_version();
----- Output 1: pm.send: can_send_thread.SubSocket.ZMQSubSocket.receive.zmq_msg_recv.Panda.can_send()
    void can_send(capnp::List<cereal::CanData>::Reader can_data_list);
    int can_receive(cereal::Event::Builder &event);

/OP081/selfdrive/boardd/panda.cc
  #include "panda.h"
  int Panda::usb_bulk_write(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout) {
    int transferred = 0;
    pthread_mutex_lock(&usb_lock);
    do {  // Try sending can messages. If the receive buffer on the panda is full it will NAK (not acknowledge)
          // and libusb will try again. After 5ms, it will time out. We will drop the messages.
      err = libusb_bulk_transfer(dev_handle, endpoint, data, length, &transferred, timeout);
      if (err == LIBUSB_ERROR_TIMEOUT) {
        LOGW("Transmit buffer full");  break;
      } else if (err != 0 || length != transferred) {
        handle_usb_issue(err, __func__);}
    } while(err != 0 && connected);
    pthread_mutex_unlock(&usb_lock);
    return transferred;}
  int Panda::usb_bulk_read(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout) {
    int transferred = 0;
    do {  err = libusb_bulk_transfer(dev_handle, endpoint, data, length, &transferred, timeout);
      if (err == LIBUSB_ERROR_TIMEOUT) {
        break; // timeout is okay to exit, recv still happened
      } else if (err == LIBUSB_ERROR_OVERFLOW) {
        LOGE_100("overflow got 0x%x", transferred);
      } else if (err != 0) {
        handle_usb_issue(err, __func__);}
    } while(err != 0 && connected);
    return transferred;}
----- Output 1: pm.send: can_send_thread.SubSocket.ZMQSubSocket.receive.zmq_msg_recv.Panda.can_send.memcpy(&,,).usb_bulk_write(3, data, size, 5)
  void Panda::can_send(capnp::List<cereal::CanData>::Reader can_data_list){
    int msg_count = can_data_list.size();
    uint32_t *send = new uint32_t[msg_count*0x10]();
    for (int i = 0; i < msg_count; i++) {
      send[i*4] = (cmsg.getAddress() << 21) | 1;
      assert(can_data.size() <= 8);
      send[i*4+1] = can_data.size() | (cmsg.getSrc() << 4);
      memcpy(&send[i*4+2], can_data.begin(), can_data.size());
    usb_bulk_write(3, (unsigned char*)send.data(), send.size(), 5);
  int Panda::can_receive(cereal::Event::Builder &event){
    uint32_t data[RECV_SIZE/4];
    int recv = usb_bulk_read(0x81, (unsigned char*)data, RECV_SIZE);
    // Not sure if this can happen
    if (recv < 0) recv = 0;
    if (recv == RECV_SIZE) {  LOGW("Receive buffer full");}
    size_t num_msg = recv / 0x10;
    auto canData = event.initCan(num_msg);
    for (int i = 0; i < num_msg; i++) { // populate message
      if (data[i*4] & 4) {
        canData[i].setAddress(data[i*4] >> 3);  // extended
        //printf("got extended: %x\n", data[i*4] >> 3);
      } else {
        canData[i].setAddress(data[i*4] >> 21);  // normal}
      canData[i].setBusTime(data[i*4+1] >> 16);
      int len = data[i*4+1]&0xF;
      canData[i].setDat(kj::arrayPtr((uint8_t*)&data[i*4+2], len));
      canData[i].setSrc((data[i*4+1] >> 4) & 0xff);}
    return recv;}

--- pthread_mutex_lock locks the mutex object referenced by mutex. If the mutex
      is already locked, then the calling thread blocks until it has acquired the mutex.
    A mutex is a mutually exclusive flag.
    https://linux.die.net/man/3/pthread_mutex_lock
--- usb_bulk_read: Read data from a bulk endpoint
    int usb_bulk_read(usb_dev_handle *dev, int ep, char *bytes, int size, int timeout);
      usb_bulk_read performs a bulk read request to the endpoint specified by ep.
      Returns number of bytes read on success or < 0 on error.
    USB-C cables can be used to quickly charge many popular devices and transfer data faster than any other USB type.
    http://193.232.24.20/doc/libusb-0.1.12/html/function.usbbulkread.html
    https://comma.ai/setup
--- libusb_bulk_transfer(dev_handle, endpoint, data, length, &transferred, timeout)
      performs a USB bulk transfer and writes the data at the specified endpoint.
    A USB bulk endpoint can transfer large amounts of data.
    Endpoints can be described as sources or sinks of data.
    As the data is flowing out from the bus host, it will end up in the EP1 OUT buffer.
    Firmware will then at its leisure read this data.
    Firmware writes (sends) data to EP1 IN which sits in the buffer until such time
      when the host sends a IN packet to that endpoint requesting the data.
    Check the transferred parameter for bulk writes. Not all of the data may have been written.
    Parameters
      dev_handle	a handle for the device to communicate with
      endpoint	the address of a valid endpoint to communicate with
      data	a suitably-sized data buffer for either input or output (depending on endpoint)
      length	for bulk writes, the number of bytes from data to be sent. for bulk reads, the maximum number of bytes to receive into the data buffer.
      transferred	output location for the number of bytes actually transferred. Since version 1.0.21 (LIBUSB_API_VERSION >= 0x01000105), it is legal to pass a NULL pointer if you do not wish to receive this information.
      timeout	timeout (in milliseconds) that this function should wait before giving up due to no response being received. For an unlimited timeout, use value 0.
    Returns
      0 on success (and populates transferred)
      LIBUSB_ERROR_TIMEOUT if the transfer timed out (and populates transferred)
      LIBUSB_ERROR_PIPE if the endpoint halted
      LIBUSB_ERROR_OVERFLOW if the device offered more data, see Packets and overflows
      LIBUSB_ERROR_NO_DEVICE if the device has been disconnected
      LIBUSB_ERROR_BUSY if called from event handling context
      LIBUSB_ERROR_INVALID_PARAM if the transfer size is larger than the operating system and/or hardware can support (see Transfer length limitations)
      another LIBUSB_ERROR code on other failures
    https://www.beyondlogic.org/usbnutshell/usb4.shtml
    https://libusb.sourceforge.io/api-1.0/group__libusb__syncio.html#ga2f90957ccc1285475ae96ad2ceb1f58c
