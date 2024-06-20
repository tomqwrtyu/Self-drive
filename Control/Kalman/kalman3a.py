"""   JLL, 2022.7.18 - 8.17
a saved kalman3.py

(OP082) jinn@Liu:~/openpilot/selfdrive/controls/tests$ python kalman3.py
Input:
  /home/jinn/dataB/**/radar/value, t, frame_times
Output:
  ???
"""
import os, glob
import numpy as np
from collections import defaultdict, deque

from kalman2 import RadarInterface, KalmanParams, Track

Vpattern = '/home/jinn/dataB/**/radar/value'
Tpattern = '/home/jinn/dataB/**/radar/t'
Fpattern = '/home/jinn/dataB/**/frame_times'
Vfiles = []
Tfiles = []
Ffiles = []
for fname in glob.glob(Vpattern, recursive=True):
  if os.path.isfile(fname):
    Vfiles.append(fname)
for fname in glob.glob(Tpattern, recursive=True):
  if os.path.isfile(fname):
    Tfiles.append(fname)
for fname in glob.glob(Fpattern, recursive=True):
  if os.path.isfile(fname):
    Ffiles.append(fname)

  #print('#--- # of CPU cores =', os.cpu_count())  # for common.realtime
    # Threads and Processes: https://medium.com/@bfortuner/python-multithreading-vs-multiprocessing-73072ce5600b
    # cpu affinity:https://docs.nersc.gov/jobs/affinity/ Pinning processes to specific cpu cores (a.k.a. cpu affinity) is important both for performance analysis and improvement.
  #print('#--- len(os.sched_getaffinity(0)) =', len(os.sched_getaffinity(0)))
print('#--- Vfile =', Vfiles[1])
radar_vls = np.load(Vfiles[1])
radar_tms = np.load(Tfiles[1])
frame_tms = np.load(Ffiles[1])  # timestamps of video frames in boot time (s)
  #--- radar_vls.shape = (19192, 7)  # 19192/60 = 20x16 = 320 Hz
  #--- radar_tms.shape = (19192,)
  #--- frame_tms.shape = (1200,)
  #--- radar_tms = [1780.836002 1780.838738 1780.841474 ... 1840.798491]
  #--- frame_tms = [1780.814717 1780.864724 1780.914717 ... 1840.76388 ]  # dt = 0.05
  #--- len(radar_tms)/len(frame_tms) = 15.993, 16 radar pts for each frame
rvls_can = np.zeros([16, 7])  # at most 16 pts [ids] of radar values per frame for Kalman tracking

  # do RadarInterface(), RadarD()
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
  v_ego = 20.  # OP079C2/selfdrive/debug/mpc/tune_longitudinal.py
  v_ego_hist.append(v_ego)
    #--- v_ego_hist = deque([0, 20.0], maxlen=2)

    # Kalman iteration index? pt.trackId? tracks.keys()? or 16 radar pts? 2 seconds?
  for j in range(16):  # set j
    rvls_can[j] = radar_vls[idx+j]
    if i == 0 and j == 0:
      print('#--- i, j, radar_tms[idx+j], radar_vls[idx+j][5], radar_vls[idx+j][6] =', i, j, radar_tms[idx+j], radar_vls[idx+j][5], radar_vls[idx+j][6])
        #--- i, j, radar_tms[idx+j], radar_vls[idx+j][5], radar_vls[idx+j][6] = 0 15 1785.800757276 1093.0 0.0
    if i == 1199 and j == 0:
      print('#--- i, j, radar_tms[idx+j], radar_vls[idx+j][5], radar_vls[idx+j][6] =', i, j, radar_tms[idx+j], radar_vls[idx+j][5], radar_vls[idx+j][6])

  rr = RI.update(rvls_can)  # can => radar raw (rr)
    #--- rr = ( points = [(trackId=1072,dRel=62.075,yRel=0.4375,vRel=-1.28125,aRel=nan,yvRel=nan,measured=false), ..., (trackId=1093, ...)]
    #print('#--- len(rr.points) =', len(rr.points))
    #--- len(rr.points) = 16

  ar_pts = {}  # all radar points
  for pt in rr.points:  # can => radar raw (rr) => points (ar_pts), trackId
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
    if i == 0 or i == 1199:
      print('#--- ids, rpt[0]x =', ids, rpt[0])
        #print('#--- ids, v_lead, rpt[2]v, rpt[1]y, rpt[0]x =', ids, v_lead, rpt[2], rpt[1], rpt[0])
        #--- ids, v_lead, rpt[2]v, rpt[1]y, rpt[0]x = 1072 21.28125 1.28125 1.5 64.88749694824219

      # create the track if it doesn't exist or it's a new track
    if ids not in tracks:
      tracks[ids] = Track(v_lead, kalman_params)  # can => rr => ar_pts => rpt => v_lead => Track(v_lead, kalman) => tracks
    tracks[ids].update(rpt[0], rpt[1], rpt[2], v_lead, rpt[3])

'''
from kalman3.py
--- 22.09.14: we don't need Ratekeeper and RadarD
from common.realtime import Ratekeeper
rk = Ratekeeper(1.0 / radar_ts, print_delay_threshold=None)
print('#--- rk._print_delay_threshold =', rk._print_delay_threshold)
print('#--- rk.monitor_time() =', rk.monitor_time())
print('#--- rk.frame =', rk.frame)
print('#--- rk.remaining =', rk.remaining)

  #print('#--- # of CPU cores =', os.cpu_count())  # for rk = Ratekeeper
    # Threads and Processes: https://medium.com/@bfortuner/python-multithreading-vs-multiprocessing-73072ce5600b
    # cpu affinity:https://docs.nersc.gov/jobs/affinity/ Pinning processes to specific cpu cores (a.k.a. cpu affinity) is important both for performance analysis and improvement.
  #print('#--- len(os.sched_getaffinity(0)) =', len(os.sched_getaffinity(0)))

'''
