#!/usr/bin/env python3
# pylint: skip-file
# flake8: noqa
# type: ignore
"""
from /openpilot/selfdrive/debug/internal/run_paramsd_on_route.py
JLL 230702
(sconsvenv) jinn@Liu:~/openpilot$ python run_paramsd_on_routeJLL.py
Output: /home/jinn/openpilot/aFigsOuts/Fig_run_paramsd_on_routeJLL.png
"""

import math
import multiprocessing
import numpy as np
from tqdm import tqdm

from selfdrive.locationd.paramsd import ParamsLearner, States
from tools.lib.logreader import LogReader
from tools.lib.route import Route

#ROUTE = "b2f1615665781088|2021-03-14--17-27-47"  # remote
ROUTE = "8bfda98c9c9e4291|2020-05-11--03-00-57"  # local  JLL
PLOT = True


def load_segment(segment_name):
  print(f"Loading {segment_name}")
    # Loading None ...
    # Loading /home/jinn/dataC/8bfda98c9c9e4291|2020-05-11--03-00-57--61/rlog.bz2
  if segment_name is None:
    return []

  try:
    return list(LogReader(segment_name))
      # bz2toh5.py: lr = LogReader(f) <= ents <= event_read_multiple_bytes() <= capnp_log <= log = capnp.load()
  except ValueError as e:
    print(f"Error parsing {segment_name}: {e}")
    return []


if __name__ == "__main__":
    #route = Route(ROUTE)
  route = Route(ROUTE, "/home/jinn/dataC")

  msgs = []
  with multiprocessing.Pool(24) as pool:
      # A process pool controls a pool of worker processes to which jobs can be submitted.
      # Google: Multiprocessing Pool.map() in Python
      # You can apply a function to each item in an iterable in parallel using the Pool map() method.
      # pool.map issues tasks for execution: results = pool.map(task, items)
      # map() function returns an iterable of return values from the task function
    for d in pool.map(load_segment, route.log_paths()):
      msgs += d
        #--- load_segment = <function load_segment at 0x7fbbda3035e0>
        #--- route.log_paths() = [None, ..., None, '/home/jinn/dataC/8bfda98c9c9e4291|2020-05-11--03-00-57--61/rlog.bz2']
        #print("#--- d =", d)
        # UnicodeDecodeError: 'utf-8' codec can't decode byte 0xcf in position 22249:

  for m in msgs:
      #--- m.which() = can
      #--- m.which() = sendcan
      #--- m.which() = liveParameters
      #--- m.which() = cameraOdometry
      #--- m.which() = lateralPlan
      #--- m.which() = controlsState
      #--- m.which() = model
      #--- m.which() = carState
      #--- m.which() = carControl ...
    if m.which() == 'carParams':
      CP = m.carParams
      '''
      print("#--- CP =", CP)
      #--- CP = ( carName = "toyota",
        carFingerprint = "TOYOTA PRIUS 2017",
        enableGasInterceptor = false, pcmCruise = true, enableCameraDEPRECATED = true,
        enableDsu = false, enableApgsDEPRECATED = false, minEnableSpeed = -1,
        minSteerSpeed = 0, safetyModelDEPRECATED = toyota, safetyParamDEPRECATED = 66,
        steerMaxBPDEPRECATED = [0], steerMaxVDEPRECATED = [1], gasMaxBPDEPRECATED = [0],
        gasMaxVDEPRECATED = [0.5], brakeMaxBPDEPRECATED = [0], brakeMaxVDEPRECATED = [1],
        mass = 1517.1876, wheelbase = 2.7, centerToFront = 1.188, steerRatio = 15.74,
        steerRatioRear = 0, rotationalInertia = 2594.37, tireStiffnessFront = 118570.52,
        tireStiffnessRear = 147271,
        longitudinalTuning = ( kpBP = [0, 5, 35], kpV = [3.6, 2.4, 1.5], kiBP = [0, 35],
          kiV = [0.54, 0.36], deadzoneBP = [0, 9], deadzoneV = [0, 0.15], kf = 0 ),
        lateralTuning = ( indi = ( outerLoopGainDEPRECATED = 3, innerLoopGainDEPRECATED = 4,
            timeConstantDEPRECATED = 1, actuatorEffectivenessDEPRECATED = 1 ) ),
        steerLimitAlert = false, vEgoStopping = 0, directAccelControlDEPRECATED = false,
        stoppingControl = false, startAccel = 0, steerRateCostDEPRECATED = 1,
        steerControlType = torque, radarUnavailable = false, steerActuatorDelay = 0.5,
        openpilotLongitudinalControl = false, carVin = "JTDKB3FU503090825", isPandaBlackDEPRECATED = true,
        dashcamOnly = false, safetyModelPassiveDEPRECATED = silent, transmissionType = unknown,
        carFw = [
          ( ecu = engine,
            fwVersion = "\0028966347B0000\000\000\000\0008966A4703000\000\000\000\000",
            address = 1792, subAddress = 0, responseAddress = 0, bus = 0, logging = false,
            obdMultiplexing = false ),
          ( ecu = abs,
            fwVersion = "F152647300\000\000\000\000\000\000",
            address = 1968, subAddress = 0, responseAddress = 0, bus = 0, logging = false,
            obdMultiplexing = false ),
          ( ecu = dsu,
            fwVersion = "881514706100\000\000\000\000",
            address = 1937, subAddress = 0, responseAddress = 0, bus = 0, logging = false,
            obdMultiplexing = false ),
          ( ecu = eps,
            fwVersion = "8965B47060\000\000\000\000\000\000",
            address = 1953, subAddress = 0, responseAddress = 0, bus = 0, logging = false,
            obdMultiplexing = false ),
          ( ecu = fwdRadar,
            fwVersion = "8821F4702300\000\000\000\000",
            address = 1872, subAddress = 15, responseAddress = 0, bus = 0, logging = false,
            obdMultiplexing = false ),
          ( ecu = fwdCamera,
            fwVersion = "8646F4705200\000\000\000\000",
            address = 1872, subAddress = 109, responseAddress = 0, bus = 0, logging = false,
            obdMultiplexing = false ) ],
        radarTimeStep = 0.05, communityFeatureDEPRECATED = false, steerLimitTimer = 0.4,
        fingerprintSource = can, networkLocation = fwdCamera, minSpeedCanDEPRECATED = 0,
        stoppingDecelRate = 0, startingAccelRateDEPRECATED = 0, maxSteeringAngleDegDEPRECATED = 0,
        fuzzyFingerprint = false, enableBsm = false, hasStockCameraDEPRECATED = false,
        longitudinalActuatorDelayUpperBound = 0, vEgoStarting = 0, stopAccel = 0,
        longitudinalActuatorDelayLowerBound = 0, wheelSpeedFactor = 0, flags = 0,
        alternativeExperience = 0, notCar = false, maxLateralAccel = 0, autoResumeSng = false,
        startingState = false, experimentalLongitudinalAvailable = false )
      '''
      break

  params = {
    'carFingerprint': CP.carFingerprint,
    'steerRatio': CP.steerRatio,
    'stiffnessFactor': 1.0,
    'angleOffsetAverageDeg': 0.0,
  }

  for m in msgs:
    if m.which() == 'liveParameters':
      params['steerRatio'] = m.liveParameters.steerRatio
      params['angleOffsetAverageDeg'] = m.liveParameters.angleOffsetAverageDeg
      break

  print(params)
    # {'carFingerprint': 'TOYOTA PRIUS 2017', 'steerRatio': 16.474075317382812, 'stiffnessFactor': 1.0, 'angleOffsetAverageDeg': 0.3493630290031433}
  learner = ParamsLearner(CP, params['steerRatio'], params['stiffnessFactor'], math.radians(params['angleOffsetAverageDeg']))
  msgs = [m for m in tqdm(msgs) if m.which() in ('liveLocationKalman', 'carState', 'liveParameters')]
    # total m: 69061
  msgs = sorted(msgs, key=lambda m: m.logMonoTime)

  ts = []
  ts_log = []
  results = []
  results_log = []
  for m in tqdm(msgs):  # total m: 12614
    if m.which() == 'carState':
      last_carstate = m

    elif m.which() == 'liveLocationKalman':
      t = last_carstate.logMonoTime / 1e9
      learner.handle_log(t, 'carState', last_carstate.carState)

      t = m.logMonoTime / 1e9
      learner.handle_log(t, 'liveLocationKalman', m.liveLocationKalman)

      x = learner.kf.x
      sr = float(x[States.STEER_RATIO])
      st = float(x[States.STIFFNESS])
      ao_avg = math.degrees(x[States.ANGLE_OFFSET])
      ao = ao_avg + math.degrees(x[States.ANGLE_OFFSET_FAST])
      r = [sr, st, ao_avg, ao]
      if any(math.isnan(v) for v in r):
        print("NaN", t)

      ts.append(t)
      results.append(r)

    elif m.which() == 'liveParameters':
      t = m.logMonoTime / 1e9
      mm = m.liveParameters

      r = [mm.steerRatio, mm.stiffnessFactor, mm.angleOffsetAverageDeg, mm.angleOffsetDeg]
      if any(math.isnan(v) for v in r):
        print("NaN in log", t)
      ts_log.append(t)
      results_log.append(r)

  results = np.asarray(results)
  results_log = np.asarray(results_log)

  if PLOT:
    import matplotlib.pyplot as plt
    plt.figure()

    plt.subplot(3, 2, 1)
    plt.title('Kalman A')
    plt.plot(ts, results[:, 0], label='Steer Ratio')
    plt.grid()
    plt.ylim([0, 20])
    plt.legend()

    plt.subplot(3, 2, 3)
    plt.title('Kalman B')
    plt.plot(ts, results[:, 1], label='Stiffness')
    plt.ylim([0, 2])
    plt.grid()
    plt.legend()

    plt.subplot(3, 2, 5)
    plt.title('Kalman C')
    plt.plot(ts, results[:, 2], label='Angle offset (average)')
    plt.plot(ts, results[:, 3], label='Angle offset (instant)')
    plt.ylim([-5, 5])
    plt.grid()
    plt.legend()

    plt.subplot(3, 2, 2)
    plt.title('liveParameters A')
    plt.plot(ts_log, results_log[:, 0], label='Steer Ratio')
    plt.grid()
    plt.ylim([0, 20])
    plt.legend()

    plt.subplot(3, 2, 4)
    plt.title('liveParameters B')
    plt.plot(ts_log, results_log[:, 1], label='Stiffness')
    plt.ylim([0, 2])
    plt.grid()
    plt.legend()

    plt.subplot(3, 2, 6)
    plt.title('liveParameters C')
    plt.plot(ts_log, results_log[:, 2], label='Angle offset (average)')
    plt.plot(ts_log, results_log[:, 3], label='Angle offset (instant)')
    plt.ylim([-5, 5])
    plt.grid()
    plt.legend()
    plt.show()
