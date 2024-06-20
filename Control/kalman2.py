'''  JLL, 2022.7.5 - 8.16, 11.15, 2023.7.10
from /selfdrive/controls/radard.py
     /selfdrive/controls/lib/radar_helpers.py
     /selfdrive/car/toyota/radar_interface.py
for 230708, /selfdrive/controls/radard.py
(sconsvenv) jinn@Liu:~/openpilot$ python kalman2.py
'''

import numpy as np
from collections import defaultdict, deque
from cereal import car
from common.kalman.simple_kalman import KF1D


def interp(x, xp, fp):
  N = len(xp)

  def get_interp(xv):
    hi = 0
    while hi < N and xv > xp[hi]:
      hi += 1
    low = hi - 1
    return fp[-1] if hi == N and xv > xp[low] else (
      fp[0] if hi == 0 else
      (xv - xp[low]) * (fp[hi] - fp[low]) / (xp[hi] - xp[low]) + fp[low])

  return [get_interp(v) for v in x] if hasattr(x, '__iter__') else get_interp(x)


class KalmanParams():  # from /selfdrive/controls/radard.py
  def __init__(self, dt):
      # Lead Kalman Filter params, calculating K from A, C, Q, R requires the control library.
      # hardcoding a lookup table to compute K for values of radar_ts between 0.1s and 1.0s
    assert dt > .01 and dt < .1, "Radar time step must be between .01s and 0.1s"
    self.A = [[1.0, dt], [0.0, 1.0]]
    self.C = [1.0, 0.0]
      #Q = np.matrix([[10., 0.0], [0.0, 100.]])
      #R = 1e3
      #K = np.matrix([[ 0.05705578], [ 0.03073241]])
    dts = [dt * 0.01 for dt in range(1, 11)]
      # = [0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.1] => 10Hz to 100Hz
    K0 = [0.12288, 0.14557, 0.16523, 0.18282, 0.19887, 0.21372, 0.22761, 0.24069, 0.2531, 0.26491]
    K1 = [0.29666, 0.29331, 0.29043, 0.28787, 0.28555, 0.28342, 0.28144, 0.27958, 0.27783, 0.27617]
    self.K = [[interp(dt, dts, K0)], [interp(dt, dts, K1)]]

_LEAD_ACCEL_TAU = 1.5
SPEED, ACCEL = 0, 1   # Kalman filter states enum


class Track():  # from /selfdrive/controls/lib/radar_helpers.py
  def __init__(self, v_lead, kalman_params):
    self.cnt = 0
    self.aLeadTau = _LEAD_ACCEL_TAU
    self.K_A = kalman_params.A
    self.K_C = kalman_params.C
    self.K_K = kalman_params.K
    self.kf = KF1D([[v_lead], [0.0]], self.K_A, self.K_C, self.K_K)
      #x0 = [[v_lead], [0.0]]
      #--- x0[0][0], x0[1][0] = 0.0 0.0
      #print('#--- self.kf.x[SPEED][0], self.kf.x[ACCEL][0] =', self.kf.x[SPEED][0], self.kf.x[ACCEL][0])
      #--- self.kf.x[SPEED][0], x[ACCEL][0] = 0.0 0.0

      # class KF1D: AK = A-KC^t, A=[[1,dt],[0,1]]: 2x2, K=[k0,k1]^t: 2x1, C=[1,0]^t
      # [Wel06](1.9): X' = AX, X' = [v',a']^t: a priori, X: prior, B=Q=0, t: transpose
      # (1.12): X'' = X'+K(z-HX'), X'': a posteriori, z: meas, H: 1x2
      # class KF1D: X'' = AKX+Kz = AX-KC^tX+Kz = AX+K(z-C^tX) =>
      # HX' = HAX = C^tX => C^t = HA => H = C^tA^{-1} = [c0,-dtc0], c0 = 1 =>
      # HX' = v'-a'dt => v'' = v'+k0(z-v'+a'dt) = v+adt+k0(z-v-adt+adt) = v+adt+k0(z-v),
      # a''=a+k1(z-v). (1.11): K = PH^t/(HPH^t), E = [e0,e1]^t error,
      # P = EE^t = [[e0^2,e0e1],[e1e0,e1^2]]: 2x2 => PH^t = [e0^2-dte0e1,e1e0-dte1^2]^t,
      # HPH^t = e0^2-dte0e1-dte1e0+dt^2e1^2 = (e0-dte1)^2 =>
      # k0 = (e0^2-dte0e1)/(e0-dte1)^2 [no unit], k1 = (e1e0-dte1^2)/(e0-dte1)^2 [unit=a/v]

  def update(self, d_rel, y_rel, v_rel, v_lead, measured):
      # relative values, copy
    self.dRel = d_rel   # LONG_DIST
    self.yRel = y_rel   # -LAT_DIST
    self.vRel = v_rel   # REL_SPEED
    self.vLead = v_lead
    self.measured = measured   # measured or estimate

      # computed velocity and accelerations
    if self.cnt > 0:
      self.kf.update(self.vLead)

    self.vLeadK = float(self.kf.x[SPEED][0])
    self.aLeadK = float(self.kf.x[ACCEL][0])

      # Learn if constant acceleration
    if abs(self.aLeadK) < 0.5:
      self.aLeadTau = _LEAD_ACCEL_TAU
    else:
      self.aLeadTau *= 0.9

    #print('#--- self.cnt, vLeadK, aLeadK, aLeadTau =', self.cnt, self.vLeadK, self.aLeadK, self.aLeadTau)
      #--- self.cnt, vLeadK, aLeadK, aLeadTau = 0 21.28125 0.0 1.5
      #--- self.cnt, vLeadK, aLeadK, aLeadTau = 1 20.771645624999998 -0.7317218749999999 1.35

    self.cnt += 1

  def get_key_for_cluster(self):
      # Weigh y higher since radar is inaccurate in this dimension
    return [self.dRel, self.yRel*2, self.vRel]

  def reset_a_lead(self, aLeadK, aLeadTau):
    self.kf = KF1D([[self.vLead], [aLeadK]], self.K_A, self.K_C, self.K_K)
    self.aLeadK = aLeadK
    self.aLeadTau = aLeadTau

class RadarInterface():  # from selfdrive/car/toyota/radar_interface.py
  def __init__(self):
    self.pts = {}
    self.delay = 0

  def update(self, can_rvls):  # can_rvls replaces can_strings
    rr = self._update(can_rvls)
    return rr

  def _update(self, can_rvls):  # see def maybe_update_radar_points(lt, lid_overlay):
    ret = car.RadarData.new_message()  # capnp.new_message()
      #--- ret = ()
    for ii in range(len(can_rvls)):
      self.pts[ii] = car.RadarData.RadarPoint.new_message()
      self.pts[ii].trackId = int(can_rvls[ii][5])
        #self.pts[0] = car.RadarData.RadarPoint.new_message()
        #--- self.pts = {0: <car.capnp:RadarData.RadarPoint builder (trackId=0,dRel=0,yRel=0,vRel=0,aRel=0,yvRel=0,measured=false)>}
      self.pts[ii].dRel = float(can_rvls[ii][0])  # from front of car
      self.pts[ii].yRel = float(can_rvls[ii][1])  # No -? in car frame's y axis, left is positive
      self.pts[ii].vRel = float(can_rvls[ii][2])
      self.pts[ii].aRel = float('nan')
      self.pts[ii].yvRel = float('nan')
      self.pts[ii].measured = bool(int(can_rvls[ii][6]))

    ret.points = list(self.pts.values())  # Python Dictionary values() Method
      #--- ret.points = [(trackId=0, dRel=0, yRel=0, vRel=0, aRel=0, yvRel=0, measured=false)]
      #--- ret = ( points = [(trackId=0,dRel=0,yRel=0,vRel=0,aRel=0,yvRel=0,measured=false)] )
      #print('#--- ret =', ret)

    return ret


if __name__ == '__main__':  # test KalmanParams(), Track()
  radar_ts = .05  # = 20Hz, 0.025 = 40Hz
  kalman_params = KalmanParams(radar_ts)
    #--- kalman_params.A = [[1.0, .05], [0.0, 1.0]]
  print('#--- vars(kalman_params) =', vars(kalman_params))  # Python: Print an Object’s Attributes
    #--- vars(kalman_params) = {'A': [[1.0, 0.05], [0.0, 1.0]], 'C': [1.0, 0.0], 'K': [[0.19887], [0.28555]]}

  tracks = defaultdict(dict)  # Python Defaultdict: Overview and Examples: defaultdict never raises a KeyError and provides a default value for missing key.
    # = defaultdict(<class 'dict'>, {})
  v_lead = 18.
  tracks[0] = Track(v_lead, kalman_params)
    #--- tracks[0].kf.A_K_1 = 0.025
  print('#--- vars(tracks[0]) =', vars(tracks[0]))  # Python: Print an Object’s Attributes
    #--- vars(tracks[0]) = {'cnt': 0, 'aLeadTau': 1.5, 'K_A': [[1.0, 0.05], [0.0, 1.0]], 'K_C': [1.0, 0.0], 'K_K': [[0.19887], [0.28555]], 'kf': <common.kalman.simple_kalman_impl.KF1D object at 0x7fa1f8b6d3b0>}

  delay = 1
  v_ego_hist = deque([0], maxlen=delay+1)  # Deque is a better list for quicker append and pop operations
  v_ego = 20.  # /selfdrive/debug/mpc/tune_longitudinal.py
  v_ego_hist.append(v_ego)
    #--- v_ego_hist = deque([0, 20.0], maxlen=2)

  d_rel, y_rel, v_rel = 31.38000031, 0.40000001, -0.2
    # [31.38000031, 0.40000001, -0.2] from openpilot/selfdrive/controls/tests/test_clustering.py
  v_lead = v_rel + v_ego_hist[0]
  print('#--- v_lead =', v_lead)
  measured = True  # openpilot/selfdrive/car/honda/radar_interface.py

    # KF update 0
  tracks[0].update(d_rel, y_rel, v_rel, v_lead, measured)
    # KF update 1
  tracks[1] = Track(v_lead, kalman_params)
  tracks[1].update(d_rel, y_rel, v_rel, v_lead, measured)
  print('#--- list(tracks.keys()) =', list(tracks.keys()))
    #--- list(tracks.keys()) = [0, 1]
