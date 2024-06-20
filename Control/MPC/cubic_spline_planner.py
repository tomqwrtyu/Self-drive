"""
Cubic spline planner   Author: Atsushi Sakai(@Atsushi_twi)
JLL 230530
(sconsvenv) jinn@Liu:~/openpilot/aMPC$ python cubic_spline_planner.py
"""
import math
import numpy as np
import bisect


class CubicSpline1D:
  def __init__(self, h, xy):
      # h: piecewise hypotenuses in ascending order, xy: x or y coordinate of h
    dh = np.diff(h)
      #--- h =      [0, 26.92, 53.85, 58.85]
      #--- xy = y = [20.0, 30.0, 5.0, 0.0]
      #--- dh =     [26.92 26.92  5.]
    if np.any(dh < 0):
      raise ValueError("h: piecewise hypotenuses must be in ascending order")

    self.a, self.b, self.c, self.d = [], [], [], []
    self.h = h
    self.nh = len(h)  # dimension of h

      # calc coefficient a
    self.a = [ixy for ixy in xy]  #--- self.a = [20.0, 30.0, 5.0, 0.0]

      # calc coefficient c
    A = self.__calc_A(dh)
    B = self.__calc_B(dh, self.a)
    self.c = np.linalg.solve(A, B)
      #--- A = [
      # [  1.     0.     0.     0. ]
      # [ 26.92 107.70  26.92   0. ]
      # [  0.    26.92  63.85   5. ]
      # [  0.     0.    0.      1. ]]
      #--- B = [ 0. -3.89  -0.21  0. ]
      #--- self.c = [ 0. -0.03 0.01  0. ]

      # calc spline coefficient b and d
    for i in range(self.nh - 1):
      d = (self.c[i + 1] - self.c[i]) / (3.0 * dh[i])
      b = 1.0 / dh[i] * (self.a[i + 1] - self.a[i]) \
          - dh[i] / 3.0 * (2.0 * self.c[i] + self.c[i + 1])
      self.d.append(d)
      self.b.append(b)
        #--- self.b = [0.72, -0.33, -1.04]
        #--- self.d = [-0.00, 0.00, -0.00]

  def calc_spline1D(self, h):
      # 1D Cubic Spline in [Liu23b]
      # Calculate the x or y coordinate of a point s on the spline curve at a given
      # h that is a cumulative length from the start point of piecewise hypotenuses
    if h < self.h[0]:
      return None
    elif h > self.h[-1]:
      return None

    i = self.__search_index(h)
    t = h - self.h[i]
    sxy = self.a[i] + self.b[i] * t + self.c[i] * t ** 2.0 + self.d[i] * t ** 3.0
    #print("#--- i, h, h[i], t =", i, h, round(self.h[i], 2), round(t, 2), \
    #      "   a[i], b[i], c[i], d[i] =", round(self.a[i], 2), round(self.b[i], 2), round(self.c[i], 2), round(self.d[i], 2))
      #---     h = [0.0, 1.0, 2.0, 3.0, ..., 56.0, 57.0, 58.0]
      #---    ay = [20.0, 30.0, 5.0, 0.0]
      #--- self.a = [20.0, 30.0, 5.0, 0.0]
    return sxy

  def calc_first_derivative(self, h):  # Calculate first derivative dy at given h
    if h < self.h[0]:
      return None
    elif h > self.h[-1]:
      return None
    i = self.__search_index(h)
    t = h - self.h[i]
    dy = self.b[i] + 2.0 * self.c[i] * t + 3.0 * self.d[i] * t ** 2.0
    return dy

  def calc_second_derivative(self, h):  # Calculate second derivative ddy at given h
    if h < self.h[0]:
      return None
    elif h > self.h[-1]:
      return None
    i = self.__search_index(h)
    t = h - self.h[i]
    ddy = 2.0 * self.c[i] + 6.0 * self.d[i] * t
    return ddy

  def __search_index(self, h):  # Search the index of h
    return bisect.bisect(self.h, h) - 1
      # bisect.bisect(H, h) returns the index i of a sorted list H such that h >= H[i]

  def __calc_A(self, dh):  # Calculate matrix A for spline coefficient c
    A = np.zeros((self.nh, self.nh))
    A[0, 0] = 1.0
    for i in range(self.nh - 1):
      if i != (self.nh - 2):
        A[i + 1, i + 1] = 2.0 * (dh[i] + dh[i + 1])
      A[i + 1, i] = dh[i]
      A[i, i + 1] = dh[i]
    A[0, 1] = 0.0
    A[self.nh - 1, self.nh - 2] = 0.0
    A[self.nh - 1, self.nh - 1] = 1.0
    return A

  def __calc_B(self, dh, a):  # Calculate matrix B for spline coefficient c
    B = np.zeros(self.nh)
    for i in range(self.nh - 2):
      B[i + 1] = 3.0 * (a[i + 2] - a[i + 1]) / dh[i + 1]\
          - 3.0 * (a[i + 1] - a[i]) / dh[i]
    return B


class CubicSpline2D:
  def __init__(self, x, y):
    self.h = self.__calc_h(x, y)  # h: piecewise hypotenuses
      #--- x = [35.0, 10.0, 0.0, 0.0]  y = [20.0, 30.0, 5.0, 0.0]
      #--- self.h =  [0, 26.92, 53.85, 58.85]
    self.sx = CubicSpline1D(self.h, x)  # sx: x coordinate of a spine point s at given h
    self.sy = CubicSpline1D(self.h, y)  # sx: y coordinate of s
      #--- self.sy = <cubic_spline_planner.CubicSpline1D object at 0x7ffa87af1c70>

  def __calc_h(self, x, y):  # Calculate the hypotenus of (dx, dy)
    dx = np.diff(x)
    dy = np.diff(y)  #--- dx = [-25. -10.   0.]  dy = [ 10. -25.  -5.]
    dh = np.hypot(dx, dy)
    h = [0]
    h.extend(np.cumsum(dh))
    return h

  def calc_spline2D(self, h):
      # Calculate the (x, y) coordinates of a point s on the spline curve at a given
      # h that is a cumulative length from the start point of piecewise hypotenuses
    x = self.sx.calc_spline1D(h)
    y = self.sy.calc_spline1D(h)
    return x, y

  def calc_curvature(self, h):  # Calculate the curvature k of the spline at given h
    dx = self.sx.calc_first_derivative(h)
    ddx = self.sx.calc_second_derivative(h)
    dy = self.sy.calc_first_derivative(h)
    ddy = self.sy.calc_second_derivative(h)
    k = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2))
    return k

  def calc_yaw(self, h):  # Calculate the yaw angle (tangent vector) of the spline at given h
    dx = self.sx.calc_first_derivative(h)
    dy = self.sy.calc_first_derivative(h)
    yaw = math.atan2(dy, dx)
    return yaw


def calc_spline_course(x, y, dh=0.1):
    # Calculate the entire course s of piecewise splines through all
    # given (x, y) points with a spacing tick dh on cumulative hypotenuses
    #--- dh = 1.0
  sp = CubicSpline2D(x, y)
  h = list(np.arange(0, sp.h[-1], dh))
    #--- h = [0.0, 1.0, 2.0, 3.0, ..., 100.0, 101.0, 102.0]
    #--- sp.h = [0, 26.92, 53.85, 58.85]
    #--- h = [0.0, 1.0, 2.0, 3.0, ..., 56.0, 57.0, 58.0]

  rx, ry, ryaw, rk = [], [], [], []
  for i_h in h:
    ix, iy = sp.calc_spline2D(i_h)
    rx.append(ix)
    ry.append(iy)
    ryaw.append(sp.calc_yaw(i_h))
    rk.append(sp.calc_curvature(i_h))
  return rx, ry, ryaw, rk, h


def main_1d():
  print("CubicSpline1D test")
  import matplotlib.pyplot as plt
  x = np.arange(5)
  y = [1.7, -6, 5, 6.5, 0.0]
  sp = CubicSpline1D(x, y)
  xi = np.linspace(0.0, 5.0)

  plt.plot(x, y, "xb", label="Data points")
  plt.plot(xi, [sp.calc_spline1D(x) for x in xi], "r", label="Cubic spline interpolation")
  plt.grid(True)
  plt.legend()
  plt.show()


def main_2d():  # pragma: no cover
  print("CubicSpline1D 2D test")
  import matplotlib.pyplot as plt
  x = [-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0]
  y = [0.7, -6, 5, 6.5, 0.0, 5.0, -2.0]
  dh = 0.1  # [m] distance of each interpolated points

  sp = CubicSpline2D(x, y)
  h = np.arange(0, sp.h[-1], dh)

  rx, ry, ryaw, rk = [], [], [], []
  for i_h in h:
    ix, iy = sp.calc_spline2D(i_h)
    rx.append(ix)
    ry.append(iy)
    ryaw.append(sp.calc_yaw(i_h))
    rk.append(sp.calc_curvature(i_h))

  plt.subplots(1)
  plt.plot(x, y, "xb", label="Data points")
  plt.plot(rx, ry, "-r", label="Cubic spline path")
  plt.grid(True)
  plt.axis("equal")
  plt.xlabel("x [m]")
  plt.ylabel("y [m]")
  plt.legend()

  plt.subplots(1)
  plt.plot(h, [np.rad2deg(iyaw) for iyaw in ryaw], "-r", label="yaw")
  plt.grid(True)
  plt.legend()
  plt.xlabel("line length [m]")
  plt.ylabel("yaw angle [deg]")

  plt.subplots(1)
  plt.plot(h, rk, "-r", label="curvature")
  plt.grid(True)
  plt.legend()
  plt.xlabel("line length [m]")
  plt.ylabel("curvature [1/m]")

  plt.show()


if __name__ == '__main__':
  # main_1d()
  main_2d()
