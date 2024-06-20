using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cerealJLL");

using Car = import "carJLL.capnp";

@0xb659a66238cbe898;

struct Joystick {
  # convenient for debug and live tuning
  axes @0: List(Float32);
  buttons @1: List(Bool);
}

struct Event {
  logMonoTime @0 :UInt64;  # nanoseconds
  valid @1 :Bool = true;
  testJoystick @2 :Joystick;
}
