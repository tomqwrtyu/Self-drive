"""   JLL, 2022.7.18 - 11.15, 2023.7.11
from kalman2.py
for 230708, /selfdrive/controls/radard.py
(sconsvenv) jinn@Liu:~/openpilot$ python kalman3.py
Input:  /home/jinn/dataB/**/radar/value, t, frame_times
Output: /openpilot/aKalman/fig1KF2.png
"""
import os, glob
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict, deque

from kalman2 import RadarInterface, KalmanParams, Track

FTpattern = '/home/jinn/dataB/**/frame_times'
RVpattern = '/home/jinn/dataB/**/radar/value'
RTpattern = '/home/jinn/dataB/**/radar/t'
SVpattern = '/home/jinn/dataB/**/speed/value'
STpattern = '/home/jinn/dataB/**/speed/t'
FTfiles = []
RVfiles = []
RTfiles = []
SVfiles = []
STfiles = []
for fname in glob.glob(FTpattern, recursive=True):
  if os.path.isfile(fname):
    FTfiles.append(fname)
for fname in glob.glob(RVpattern, recursive=True):
  if os.path.isfile(fname):
    RVfiles.append(fname)
for fname in glob.glob(RTpattern, recursive=True):
  if os.path.isfile(fname):
    RTfiles.append(fname)
for fname in glob.glob(SVpattern, recursive=True):
  if os.path.isfile(fname):
    SVfiles.append(fname)
for fname in glob.glob(STpattern, recursive=True):
  if os.path.isfile(fname):
    STfiles.append(fname)

print('#--- RVfile[1] =', RVfiles[1])
frame_tms = np.load(FTfiles[1])  # timestamps of video frames in boot time (s)
radar_vls = np.load(RVfiles[1])
radar_tms = np.load(RTfiles[1])
speed_vls = np.load(SVfiles[1])
speed_tms = np.load(STfiles[1])
  #--- frame_tms.shape = (1200,)     # 20 Hz => dt = 0.05
  #--- radar_vls.shape = (19192, 7)  # 19192/60 = 320 Hz = 20x16 => dt = 0.0031
  #--- radar_vls = [[6.488750e+01 ...] [...] ...]
  #--- radar_tms.shape = (19192,)
  #--- radar_tms = [1780.836002 1780.838738 1780.841474 ... 1840.798491]  # dt = 0.0027, delay = 0.021?
  #--- frame_tms = [1780.814717 1780.864724 1780.914717 ... 1840.76388 ]
  #--- len(radar_tms)/len(frame_tms) = 15.993, 16 radar pts for each frame
  #--- speed_vls.shape = (2994, 1)
  #--- speed_tms.shape = (2994,)  # 2994/60 = 49.9 Hz => dt = 0.02
plt.figure()
plt.subplot(1, 4, 1)
plt.plot(radar_vls[:, 0], '.', markersize=1)
plt.title('xRel', fontsize=14)
plt.subplot(1, 4, 2)
plt.plot(radar_vls[:, 1], '.', markersize=1)
plt.title('yRel', fontsize=14)
plt.subplot(1, 4, 3)
plt.plot(radar_vls[:, 2], '.', markersize=1)
plt.title('vRel', fontsize=14)
plt.subplot(1, 4, 4)
plt.plot(speed_vls[:, 0], '.', markersize=1)
plt.title('vEgo', fontsize=14)
plt.show()

  # do RadarD()
  #can_rvls = np.zeros([16, 7])  # at most 16 pts [ids] of radar values from can per frame for Kalman tracking
radar_ts = .05  # = radarTimeStep = 20Hz = FPS, 0.025 = 40Hz;
kalman_params = KalmanParams(radar_ts)
delay = 1
tracks = defaultdict(dict)  # = defaultdict(<class 'dict'>, {}), dict: Python keyword. How does collections.defaultdict work?

RI = RadarInterface()

  #for i in range(len(frame_tms)):
  #for i in range(600, 610):  # check radar_vls[idx+j][5] = trackId
  #for i in range(1190, 1200):  # check radar_vls[idx+j][5] = trackId
  #for i in range(0, 10):
for i in range(0, 1200, 1199):  # frame i
  arr = radar_tms - frame_tms[i]
  idx = np.where(arr > 0, arr, np.inf).argmin()  # we need frame_tms[i] < radar_tms[idx]
    #print('#--- i, idx, frame_tms[i], radar_tms[idx] =', i, idx, frame_tms[i], radar_tms[idx])
    #--- i, idx, frame_tms[i], radar_tms[idx] = 99 1584 1785.764654 1785.795115099

  v_ego_hist = deque([20.], maxlen=delay+1)  # Deque is a better list for quicker append and pop operations
    # /car/honda/radar_interface.py
    # self.delay = int(round(0.1 / CP.radarTimeStep))   # 0.1s delay of radar
    # radarTimeStep @45: Float32 = 0.05;  # time delta between radar updates, 20Hz is very standard
    # 0.1 / 0.05  = 2
  v_ego = 20.  # /selfdrive/debug/mpc/tune_longitudinal.py
  v_ego_hist.append(v_ego)
    #--- v_ego_hist = deque([0, 20.0], maxlen=2)

    # Kalman iteration index j? pt.trackId=tracks.keys()=ids? or 16 radar pts? 2 seconds?
    # Answer: 2 seconds.
  can_rvls = []
  for j in range(16):  # set j
    #can_rvls[j] = radar_vls[idx+j]
    can_rvls.append(radar_vls[idx+j])
    if i == 0 and j == 0:
      print('#--- i, j, radar_tms[idx+j], can_rvls[j][0], can_rvls[j][5] =', i, j, radar_tms[idx+j], can_rvls[j][0], can_rvls[j][5])
        #--- i, j, radar_tms[idx+j], can_rvls[j][0], can_rvls[j][5] = 0 0 1780.836002299 64.8875 1072.0
    if i == 1199 and j == 0:
      print('#--- i, j, radar_tms[idx+j], can_rvls[j][0], can_rvls[j][5] =', i, j, radar_tms[idx+j], can_rvls[j][0], can_rvls[j][5])

  rr = RI.update(can_rvls)  # can => RI => radar raw (rr)
    #--- rr = ( points = [(trackId=1072, dRel=62.075, yRel=0.4375, vRel=-1.28125, aRel=nan, yvRel=nan, measured=false), ..., (trackId=1093, ...)]
    #--- len(rr.points) = 16
    #--- rr.canMonoTimes = []: not used.

  ar_pts = {}  # all radar points
  for pt in rr.points:  # can => RI => rr.points[7] => ar_pts[ids][4]
    ar_pts[pt.trackId] = [pt.dRel, pt.yRel, pt.vRel, pt.measured]
      #if i == 0:
        #print('#--- pt.trackId, ar_pts[pt.trackId] =', pt.trackId, ar_pts[pt.trackId])
          #--- pt.trackId, ar_pts[pt.trackId] = 1072 [64.88749694824219, 1.5, 1.28125, False]

    # *** remove missing points from meta data ***
  for ids in list(tracks.keys()):
    if ids not in ar_pts:
      tracks.pop(ids, None)  # collections.defaultdict.pop

    # *** compute the tracks ***
  for ids in ar_pts:
    rpt = ar_pts[ids]  # can => rr => ar_pts => rpt
      # align v_ego by a fixed time to align it with the radar measurement
    v_lead = rpt[2] + v_ego_hist[0]  # rr => ar_pts => rpt => v_lead
      #if i == 0 or i == 1199:
        #print('#--- ids, rpt[0]x =', ids, rpt[0])
        #print('#--- ids, v_lead, rpt[2]v, rpt[1]y, rpt[0]x =', ids, v_lead, rpt[2], rpt[1], rpt[0])
        #--- ids, v_lead, rpt[2]v, rpt[1]y, rpt[0]x = 1072 21.28125 1.28125 1.5 64.88749694824219

      # create the track if it doesn't exist or it's a new track
    if ids not in tracks:
      tracks[ids] = Track(v_lead, kalman_params)  # can => rr => ar_pts => rpt => v_lead => Track(v_lead, kalman) => tracks
    tracks[ids].update(rpt[0], rpt[1], rpt[2], v_lead, rpt[3])

  print('#--- len(tracks) =', len(tracks))

'''
'''
