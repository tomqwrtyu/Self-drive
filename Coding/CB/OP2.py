OP Code Notes, JLL 2020.10.13 - 2021.7.30

210219:
Calling C++ Classes from Python, with ctypes
selfdrive/manager.py
from common.params import Params
if params.get(k)
def main(): params = Params() params.manager_start()
selfdrive/common/params.cc
Params::Params(bool persistent_param)


210219:
Steering B. steerRatio: paramsd.py, plannerd.py
   B1. selfdrive/locationd/paramsd.py: main():
from selfdrive.locationd.models.car_kf import CarKalman,
class ParamsLearner:
self.kf = CarKalman(..., steer_ratio, ..., angle_offset)
learner = ParamsLearner(CP, params['steerRatio'], ...
msg = messaging.new_message('liveParameters')
x = learner.kf.x; msg.liveParameters.steerRatio = float(x ...
'steerRatio': msg.liveParameters.steerRatio
   B2. selfdrive/locationd/models/car_kf.py:
from rednose import KalmanFilter
from rednose.helpers.ekf_sym import EKF_sym, gen_code
class CarKalman(KalmanFilter): x = sp.Matrix([v, r]);
gen_code(...; self.filter = EKF_sym(
   B3. commaai/rednose/rednose/__init__.py
from rednose.helpers.ekf_sym import EKF_sym
class KalmanFilter:
self.filter = EKF_sym(...; self.filter.predict_and_update_batch(...
   B4. rednose/rednose/helpers/ekf_sym.py
(error-state Kalman filter (ESKF), EKF-SLAM with MSCKF)
def gen_code(folder, ...; if eskf_params:...; if msckf_params:
class EKF_sym(): def _predict_and_update_batch(
# The main kalman filter function
return xk_km1, xk_k, Pk_km1, Pk_k, t, kind, y, z, extra_args
   B6: selfdrive/controls/plannerd.py: main()
def plannerd_thread(sm=None, pm=None):
sm['liveParameters'].steerRatio = CP.steerRatio
   B7. selfdrive/controls/lib/lateral_planner.py
class LateralPlanner(): def publish(self, sm, pm):
plan_send.lateralPlan.steeringAngleDeg = float(self...._angle_deg)

Steering C.  output_steer
    C1: selfdrive/controls/lib/latcontrol_pid.py
>> output_steer = self.pid.update(self.angle_steers_des, CS.steeringAngle; return output_steer, float(self.angle_steers_des)
    >> steer_angle_rad = (CS.steeringAngle - .angleOffset >> selfdrive/car/toyota/carstate.py >> ret.steeringAngle; selfdrive/controls/lib/lateral_planner.py >> .angleOffset = float(sm['liveParameters'].angleOffsetAverage)

210218:
Steering A. steeringAngleDeg: controlsd.py
    A1. cereal/car.capnp
struct CarState { steeringAngleDeg@7 :Float32;
struct CarControl { actuators@6 :Actuators;
struct Actuators { steer@2: Float32; steeringAngleDeg@3: Float32;
struct CarParams { steerRatio@20 :Float32;
    A2. cereal/__init__.py
import capnp
car = capnp.load(os.path.join(CEREAL_PATH, "car.capnp"))
    A3. selfdrive/car/car_helpers.py
from cereal import car
def load_interfaces(brand_names): path = ('selfdrive.car.%s' % brand_name)
CarInterface = __import__(path + '.interface', fromlist=['...']).CarInterface
CarState = __import__(path + '.carstate', fromlist=['CarState']).CarState
CarController = __import__(path + '.carcontroller', fromlist=['CarController'])...
interfaces = load_interfaces(interface_names)
def get_car(logcan, sendcan):
CarInterface, CarController, CarState = interfaces[candidate]
return CarInterface(car_params, CarController, CarState), car_params
    A4. selfdrive/car/toyota/interface.py:
from cereal import car
from selfdrive.car.interfaces import CarInterfaceBase
class CarInterface(CarInterfaceBase):
if candidate == CAR.PRIUS: ret.lateralTuning.init('indi')
    A5. selfdrive/car/toyota/carcontroller.py:
from cereal import car
class CarController():
def update(self, enabled, CS, frame, actuators,
new_steer = int(round(actuators.steer*CarControllerParams.STEER_MAX))
    A6. common/numpy_fast.py:
def clip(x, lo, hi): return max(lo, min(hi, x))
    A7. selfdrive/controls/lib/latcontrol_indi.py:
class LatControlINDI():
def update(self, active, CS, CP, lat_plan):
self.angle_steers_des = lat_plan.steeringAngleDeg
self.output_steer = self.delayed_output + delta_u
self.output_steer = clip(self.output_steer, -steers_max, steers_max)
return float(self.output_steer), float(self.angle_steers_des), indi_log
    A8. selfdrive/controls/controlsd.py: main()
from cereal import car, log
class Controls: def __init__(self, sm=None, pm=None, can_sock=None):
elif self.CP.lateralTuning.which() == 'indi':
self.LaC = LatControlINDI(self.CP)
def state_control(self, CS):
# Given the state, this function returns an actuators packet
lat_plan = self.sm['lateralPlan']
actuators = car.CarControl.Actuators.new_message()
actuators.steer, actuators.steeringAngleDeg, lac_log = self.LaC.update(self.active, CS, self.CP, lat_plan)
return actuators, v_acc_sol, a_acc_sol, lac_log
def publish_logs(self, CS, start_time, actuators, v_acc, a_acc, lac_log):
# Send actuators and hud commands to the car, ...
steer_angle_rad = (CS.steeringAngleDeg -
dat = messaging.new_message('controlsState')
controlsState = dat.controlsState
controlsState.curvature = self.VM.calc_curvature(steer_angle_rad, CS.vEgo)
controlsState.steeringAngleDesiredDeg = float(self.LaC.angle_steers_des)
self.pm.send('controlsState', dat)
def step(self): actuators, v_acc, a_acc, lac_log = self.state_control(CS)
def controlsd_thread(self): self.step()
def main(sm=None, pm=None, logcan=None):
controls = Controls(sm, pm, logcan); controls.controlsd_thread()

201028:
Longitudinal Control: Adaptive Cruise Control, Model Predictive Control (mpc)
Pipe: car's radar -> 16 to 18 detected objects -> driving.cc -> lead car ->
      Kalman filter -> lead’s accurate speed, acceleration, distance ->
      long_mpc.py -> PI -> controls the gas and brakes
Lateral Control: Vision System Control
Pipe: Plan (now + steerActuatorDelay) -> Vehicle Model -> Desired Steer Output
  Read https://github.com/commaai/cereal/blob/master/service_list.yaml

201013:
Q1: input_dim=[['input_imgs', '1,12,128,256']?
YUV411 = 6 channels * 2 images = 12
selfdrive/camerad/transforms/rgb_to_yuv.c
* YUV color-spaces are a more efficient coding and reduce the bandwidth more than RGB capture can.The Y′UV model defines a color space in terms of one luma component (Y′) and two chrominance components, called U (blue projection) and V (red projection) respectively.  Y′UV signals are typically created from RGB (red, green and blue) sources. RGB files are typically encoded in 8, 12, 16 or 24 bits per pixel. Y′UV files can be encoded in 12, 16 or 24 bits per pixel. The common formats are YUV444, YUV411…
4:2:2: The two chroma components are sampled at half the sample rate of luma: the horizontal chroma resolution is halved.
In 4:1:1 chroma subsampling, the horizontal color resolution is quartered, and the bandwidth is halved compared to no chroma subsampling.
 YUV data formats: Each unique Y, U and V value comprises 8 bits, or one byte, of data. Where supported, our color camera models allow YUV transmission in 24-, 16-, and 12-bit per pixel (bpp) format.  In 16 and 12 bpp formats, the U and V color values are shared between pixels
The YUV444 data format transmits 24 bits per pixel.  Each pixel is assigned unique Y, U and V values—1 byte for each value.
YUV444: U0Y0V0U1Y1V1U2Y2V2…(4 pixels have 12 bytes)
The YUV422 data format shares U and V values between two pixels.
YUV422: U0Y0V0Y1U2Y2V2Y3…(4 pixels have 8 bytes)
YUV411: U0Y0Y1V0Y2Y3… (4 pixels have 6 bytes)
YUV420 is a planar format, meaning that the Y, U, and V values are grouped together instead of interspersed.
YUV420: Y0Y1Y2Y3 …U0 ...V0 (4 pixels have 6 bytes)
selfdrive/camerad/cameras/camera_common.h
* Frame_stride: What does “stride” mean in image processing?
selfdrive/camerad/main.cc
* Build_debayer_program: The debayer filter is used to convert raw image data into an RGB image. debayer = demosaic => Convert Bayer pattern encoded image to truecolor (RGB) image (matlab)
debayer_local_work_size = 128
selfdrive/camerad/imgproc/utils.h
* Lapl_conv_krnl[9], Laplace Conv Kernel,
* FULL_STRIDE_X 1280 FULL_STRIDE_Y 896?
