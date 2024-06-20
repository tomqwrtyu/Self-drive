using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cerealJLL");

@0xb4b6aeca6a643c1c;

struct CarControl {
  enabled @0 :Bool;
  actuators @1 :Actuators;
  struct Actuators {
    # range from -1.0 - 1.0
    steer @0: Float32;
    accel @1: Float32; # m/s^2
  }
}
