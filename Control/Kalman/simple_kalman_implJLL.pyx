# distutils: language = c++
# cython: language_level=3
# JLL, 2022.6.28 - 7.12  from /OP082/common/kalman/simple_kalman_impl.pyx
# [Wel06] G. Welch and G. Bishop, An Introduction to the Kalman Filter, 2006

cdef class KF1D:
  def __init__(self, x0, A, C, K):
    self.x0_0 = x0[0][0]  # SPEED, v
    self.x1_0 = x0[1][0]  # ACCEL, a
    self.A0_0 = A[0][0]   # = 1
    self.A0_1 = A[0][1]   # = dt
    self.A1_0 = A[1][0]   # = 0
    self.A1_1 = A[1][1]   # = 1
    self.C0_0 = C[0]      # = 1 = c0
    self.C0_1 = C[1]      # = 0 = c1
    self.K0_0 = K[0][0]
    self.K1_0 = K[1][0]

    self.A_K_0 = self.A0_0 - self.K0_0 * self.C0_0  # = 1 - k0
    self.A_K_1 = self.A0_1 - self.K0_0 * self.C0_1  # = dt
    self.A_K_2 = self.A1_0 - self.K1_0 * self.C0_0  # = - k1
    self.A_K_3 = self.A1_1 - self.K1_0 * self.C0_1  # = 1

  def update(self, meas):
      # class KF1D: AK = A-KC^t, A=[[1,dt],[0,1]]: 2x2, K=[k0,k1]^t: 2x1, C=[1,0]^t, t: transpose
      # [Wel06](1.9): X' = AX0 = [v0+dta0,a0]^t: a priori, X0=[v0,a0]^t: prior (0: initial), B=Q=R=0
      # (1.10): P' = AP0A^t, P0=I identity, A^t=[[1,0],[dt,1]]
      # (1.10): P' = [[1+dt^2,dt],[dt,1]] => (1.11): K1 = P'H^t/(HP'H^t), H = ? =>
      # (1.12): X'' = X'+K(z-HX'), X'': a posteriori, z: meas, H: 1x2
      # class KF1D: X'' = AKX+Kz = AX-KC^tX+Kz = AX+K(z-C^tX) => HX' = HAX = C^tX =>
      # C^t = HA => A^-1=[[1,-dt],[0,1]], H = C^tA^-1 = [1,-dt] =>
      # (1.11): P'H^t = [1,0]^t, HP'H^t = 1, K1 = P'H^t/(HP'H^t) = [1,0]^t
      # (1.12): K1(z0-HX') = [z0-HX',0]^t, HX' = v0-a0dt =>
      # X'' = AX+K1(z0-HX') = [v0+dta0,a0]^t + [z0-HX',0]^t = [z0+dta0,a0]^t = X1 =>
      # (1.13): K1H = [[1,-dt],[0,0]], K1HP' = [[1,0],[0,0]], P1 = P'-K1HP' = [[dt^2,dt],[dt,1]]
      # (1.9): X' = AX = [z0+2dta0,a0]^t
      # (1.10): P' = AP1A^t = A[[2dt^2,dt],[2dt,1]] = [[4dt^2,2dt],[2dt,1]]
      # (1.11): P'H^t = [2dt^2,dt]^t, HP'H^t = dt^2, K2 = [2,1/dt]^t, 2>1?

      # E = [e0,e1]^t error, e0' = v-v', e1' = a-a' = 0, e0'' = v-v'', e1'' = a-a'' = 0
      # P = EE^t = [[e0^2,e0e1],[e1e0,e1^2]] = [[e00,e01],[e10,e11]]: 2x2
      # PH^t = [e0^2-dte0e1,e1e0-dte1^2]^t,
      # HPH^t = e0^2-dte0e1-dte1e0+dt^2e1^2 = (e0-dte1)^2 => a' = a, v' = v+dta,
      # k0 = (e0^2-dte0e1)/(e0-dte1)^2 [no unit], k1 = (e1e0-dte1^2)/(e0-dte1)^2 [unit=a/v]
      # dt = 0.05 = 20Hz => L'Hôpital e1->0 => k0 = -dte0/(2(e0-dte1))(-dt) = 1/2 or k0 =
      # (2e0-dte1)/(2(e0-dte1)) = 1/2 = k1 => k0 = 0.19887, k1 = 0.28555 by code?
    cdef double x0_0 = self.A_K_0 * self.x0_0 + self.A_K_1 * self.x1_0 + self.K0_0 * meas
    cdef double x1_0 = self.A_K_2 * self.x0_0 + self.A_K_3 * self.x1_0 + self.K1_0 * meas
    self.x0_0 = x0_0
    self.x1_0 = x1_0

    return [self.x0_0, self.x1_0]

  @property
  def x(self):
    return [[self.x0_0], [self.x1_0]]

  @x.setter
  def x(self, x):
    self.x0_0 = x[0][0]
    self.x1_0 = x[1][0]
