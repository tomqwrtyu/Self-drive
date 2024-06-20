'''  JLL  2022.7.12, 2023.7.12
from /openpilot/common/kalman/tests/test_simple_kalman.py
for 230708, /selfdrive/controls/radard.py
read KF2 [Liu23c], KF1 => KF2:
  class KF1D: X0=[v0,a0]^t: 2x1, A=[[1,dt],[0,1]]: 2x2, K=[k0,k1]^t: 2x1, C=[1,0]: 1x2, AK = A-KC
  (1.1): X' = AX0 = [v0+dta0,a0]^t: a priori, X0: previous a posteriori, B=V=W=0, t: transpose
  (1.2): E' = AE0A^t = [[1+dt^2,dt],[dt,1]]: 2x2, E0=I identity, A^t=[[1,0],[dt,1]], dt = 0.01
  (2.1): K1 = E'M^t/(ME'M^t), M = ?, X1: current a posteriori, z: meas
  (2.2): X1 = X'+K1(z1-MX') = AX0+K1(z1-MAX0) = AX0+K1(z1-MAX0) = (A-K1MA)X0+K1z1 => C = MA =>
         => A^-1=[[1,-dt],[0,1]] => M = CA^-1 = [1,-dt]: 1x2 => z1: 1x1
  (2.3): E1 = (I-K1M)E'

(sconsvenv) jinn@Liu:~/openpilot$ python kalman2a.py
'''

import random
import numpy as np
from common.kalman.simple_kalman import KF1D

#dt = 0.01  # Case 1
dt = 0.05  # Case 2: Toyoyta: radarTimeStep = radar_ts = 0.05

class SimpleKalman():
  def __init__(self):
    x0_0 = 0.0
    x1_0 = 0.0
    A0_0 = 1.0
    A0_1 = dt
    A1_0 = 0.0
    A1_1 = 1.0
    C0_0 = 1.0
    C0_1 = 0.0
    #K0_0 = 0.12287673  # Case 1
    #K1_0 = 0.29666309
    K0_0 = 0.1988689  # Case 2: KF2 [Liu23c]
    K1_0 = 0.28555364

    self.kf = KF1D(x0=[[x0_0], [x1_0]],
                   A=[[A0_0, A0_1], [A1_0, A1_1]],
                   C=[C0_0, C0_1],
                   K=[[K0_0], [K1_0]])

def main():
  SK = SimpleKalman()
    #--- SK.kf.x = [[0.0], [0.0]]
  v_meas = 10.
  for i in range(200):
      #v_meas = random.uniform(0, 20)
    x = SK.kf.update(v_meas)
    z = x[0] - x[1] * dt  # see KF2 [Liu23c]
    print('#--- i, v_meas, x, z =', i, round(v_meas, 2), [ round(elem, 2) for elem in x ], round(z, 2))
      #--- Case 1
      #--- i, v_meas, x, z = 0 10.0 [1.23, 2.97] 1.2
      #--- i, v_meas, x, z = 1 10.0 [2.34, 5.57] 2.28 ...
      #--- i, v_meas, x, z = 198 10.0 [10.01, 0.07] 10.01
      #--- i, v_meas, x, z = 199 10.0 [10.01, 0.06] 10.01 => lead car: constant speed, zero accel
      #--- Case 2
      #--- i, v_meas, x, z = 0 10.0 [1.99, 2.86] 1.85
      #--- i, v_meas, x, z = 1 10.0 [3.72, 5.14] 3.47
      #--- i, v_meas, x, z = 77 10.0 [9.99, -0.01] 9.99
      #--- i, v_meas, x, z = 78 10.0 [10.0, -0.01] 10.0
      #--- i, v_meas, x, z = 198 10.0 [10.0, 0.0] 10.0
      #--- i, v_meas, x, z = 199 10.0 [10.0, 0.0] 10.0

if __name__ == "__main__":
  main()
