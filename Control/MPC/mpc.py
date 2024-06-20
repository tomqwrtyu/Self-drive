"""
mpc.py = model_predictive_speed_and_steer_control.py
Path tracking simulation with iterative linear model predictive control for speed and steer control
author: Atsushi Sakai (@Atsushi_twi)
JLL 230530
(sconsvenv) jinn@Liu:~/openpilot/aMPC$ pip install --upgrade pip
(sconsvenv) jinn@Liu:~/openpilot/aMPC$ pip install cvxpy
(sconsvenv) jinn@Liu:~/openpilot/aMPC$ python mpc.py
"""
import matplotlib.pyplot as plt
import cvxpy
import math
import numpy as np
import sys
import pathlib
#sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

import cubic_spline_planner

NS = 4  # S = x, y, v, yaw
NU = 2  # U = [accel, steer]
T = 5  # horizon length

# mpc parameters
Rb = np.diag([0.01, 0.01])  # input cost matrix
R = np.diag([0.01, 1.0])  # input difference cost matrix
Q = np.diag([1.0, 1.0, 0.5, 0.5])  # state cost matrix
GOAL_DIS = 1.5  # goal distance
STOP_SPEED = 0.5 / 3.6  # stop speed
MAX_TIME = 500.0  # max simulation time

# iterative paramter
MAX_ITER = 3  # Max iteration
DU_TH = 0.1  # iteration finish param

TARGET_SPEED = 10.0 / 3.6  # [m/s] target speed
N_IND_SEARCH = 10  # Search index number

DT = 0.2  # [s] time tick

# Vehicle parameters
LENGTH = 4.5  # [m]
WIDTH = 2.0  # [m]
BACKTOWHEEL = 1.0  # [m]
WHEEL_LEN = 0.3  # [m]
WHEEL_WIDTH = 0.2  # [m]
TREAD = 0.7  # [m]
WB = 2.5  # [m]

MAX_STEER = np.deg2rad(45.0)  # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(30.0)  # maximum steering speed [rad/s]
MAX_SPEED = 55.0 / 3.6  # maximum speed [m/s]
MIN_SPEED = -20.0 / 3.6  # minimum speed [m/s]
MAX_ACCEL = 1.0  # maximum accel [m/ss]

show_animation = True


class State:  # vehicle state class
  def __init__(self, x=0.0, y=0.0, v=0.0, yaw=0.0):
    self.x = x
    self.y = y
    self.v = v
    self.yaw = yaw
    self.predelta = None


def pi_2_pi(angle):
  while(angle > math.pi):
    angle = angle - 2.0 * math.pi
  while(angle < -math.pi):
    angle = angle + 2.0 * math.pi
  return angle


def get_linear_model_matrix(v, phi, delta):
  A = np.zeros((NS, NS))
  A[0, 0] = 1.0
  A[1, 1] = 1.0
  A[2, 2] = 1.0
  A[3, 3] = 1.0
  A[0, 2] = DT * math.cos(phi)
  A[0, 3] = - DT * v * math.sin(phi)
  A[1, 2] = DT * math.sin(phi)
  A[1, 3] = DT * v * math.cos(phi)
  A[3, 2] = DT * math.tan(delta) / WB

  B = np.zeros((NS, NU))
  B[2, 0] = DT
  B[3, 1] = DT * v / (WB * math.cos(delta) ** 2)

  C = np.zeros(NS)
  C[0] = DT * v * math.sin(phi) * phi
  C[1] = - DT * v * math.cos(phi) * phi
  C[3] = - DT * v * delta / (WB * math.cos(delta) ** 2)

  return A, B, C


def plot_car(x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):  # pragma: no cover
  outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                      [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])
  fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                       [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])
  rr_wheel = np.copy(fr_wheel)
  fl_wheel = np.copy(fr_wheel)
  fl_wheel[1, :] *= -1
  rl_wheel = np.copy(rr_wheel)
  rl_wheel[1, :] *= -1

  Rot1 = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
  Rot2 = np.array([[math.cos(steer), math.sin(steer)], [-math.sin(steer), math.cos(steer)]])

  fr_wheel = (fr_wheel.T.dot(Rot2)).T
  fl_wheel = (fl_wheel.T.dot(Rot2)).T
  fr_wheel[0, :] += WB
  fl_wheel[0, :] += WB
  fr_wheel = (fr_wheel.T.dot(Rot1)).T
  fl_wheel = (fl_wheel.T.dot(Rot1)).T

  outline = (outline.T.dot(Rot1)).T
  rr_wheel = (rr_wheel.T.dot(Rot1)).T
  rl_wheel = (rl_wheel.T.dot(Rot1)).T

  outline[0, :] += x
  outline[1, :] += y
  fr_wheel[0, :] += x
  fr_wheel[1, :] += y
  rr_wheel[0, :] += x
  rr_wheel[1, :] += y
  fl_wheel[0, :] += x
  fl_wheel[1, :] += y
  rl_wheel[0, :] += x
  rl_wheel[1, :] += y

  plt.plot(np.array(outline[0, :]).flatten(), np.array(outline[1, :]).flatten(), truckcolor)
  plt.plot(np.array(fr_wheel[0, :]).flatten(), np.array(fr_wheel[1, :]).flatten(), truckcolor)
  plt.plot(np.array(rr_wheel[0, :]).flatten(), np.array(rr_wheel[1, :]).flatten(), truckcolor)
  plt.plot(np.array(fl_wheel[0, :]).flatten(), np.array(fl_wheel[1, :]).flatten(), truckcolor)
  plt.plot(np.array(rl_wheel[0, :]).flatten(), np.array(rl_wheel[1, :]).flatten(), truckcolor)
  plt.plot(x, y, "*")


def get_nparray_from_matrix(x):
  return np.array(x).flatten()


def calc_nearest_index(state, cx, cy, cyaw, pind):
  dx = [state.x - icx for icx in cx[pind:(pind + N_IND_SEARCH)]]
  dy = [state.y - icy for icy in cy[pind:(pind + N_IND_SEARCH)]]
  d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

  mind = min(d)
  ind = d.index(mind) + pind  # ind >= pind
  mind = math.sqrt(mind)

  dxl = cx[ind] - state.x
  dyl = cy[ind] - state.y
  angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))

  if angle < 0:
      #print("#--- angle, mind, state.x =", round(angle, 2), round(mind, 2), round(state.x, 2))
      #--- angle, mind, state.x = -3.14 0.12 0.12
      #--- angle, mind, state.x = -1.76 1.65 31.15
      #--- angle, mind, state.x = -2.11 0.55 33.92
      #--- angle, mind, state.x = -3.09 0.08 -0.01
    mind *= -1

  return ind, mind


def UpdateState(state, a, delta):
  # input check
  if delta >= MAX_STEER:
    delta = MAX_STEER
  elif delta <= -MAX_STEER:
    delta = -MAX_STEER

  state.x = state.x + state.v * math.cos(state.yaw) * DT
  state.y = state.y + state.v * math.sin(state.yaw) * DT
  state.yaw = state.yaw + state.v / WB * math.tan(delta) * DT
  state.v = state.v + a * DT

  if state.v > MAX_SPEED:
    state.v = MAX_SPEED
  elif state.v < MIN_SPEED:
    state.v = MIN_SPEED

  return state


def PredictState(Sk, oa, od, Sref):
  Sbar = Sref * 0.0
  for i, _ in enumerate(Sk):
    Sbar[i, 0] = Sk[i]

  state = State(x=Sk[0], y=Sk[1], v=Sk[2], yaw=Sk[3])
  for (ai, di, i) in zip(oa, od, range(1, T + 1)):
    state = UpdateState(state, ai, di)
    Sbar[0, i] = state.x
    Sbar[1, i] = state.y
    Sbar[2, i] = state.v
    Sbar[3, i] = state.yaw

  return Sbar


def RefTrajectory(state, cx, cy, cyaw, ck, cv, dl, pind):
  Sref = np.zeros((NS, T + 1))
  dref = np.zeros((1, T + 1))
  ncourse = len(cx)

  ind, _ = calc_nearest_index(state, cx, cy, cyaw, pind)
  if ind <= pind:
      #--- Is ind <= pind possible? Yes if printed.  # Yes, due to ind = pind.
    ind = pind

  Sref[0, 0] = cx[ind]
  Sref[1, 0] = cy[ind]
  Sref[2, 0] = cv[ind]
  Sref[3, 0] = cyaw[ind]
  dref[0, 0] = 0.0  # steer operational point should be 0

  travel = 0.0
  for i in range(T + 1):
    travel += abs(state.v) * DT
    dind = int(round(travel / dl))

    if (ind + dind) < ncourse:
      Sref[0, i] = cx[ind + dind]
      Sref[1, i] = cy[ind + dind]
      Sref[2, i] = cv[ind + dind]
      Sref[3, i] = cyaw[ind + dind]
      dref[0, i] = 0.0
    else:
      Sref[0, i] = cx[ncourse - 1]
      Sref[1, i] = cy[ncourse - 1]
      Sref[2, i] = cv[ncourse - 1]
      Sref[3, i] = cyaw[ncourse - 1]
      dref[0, i] = 0.0

  print("#--- state.v, travel =", round(state.v, 2), round(travel, 2))
    #--- state.v, travel = 0.0 0.0
    #--- state.v, travel = 0.2 0.24
    #--- state.v, travel = 5.28 6.33
    #--- state.v, travel = -0.14 0.16
    #--- state.v, travel = -4.49 5.39
    #--- state.v, travel = 0.16 0.19
  return Sref, ind, dref


def NewtonIter(Sref, Sk, dref, oa, od):
  ox, oy, oyaw, ov = None, None, None, None

  if oa is None or od is None:
    oa = [0.0] * T
    od = [0.0] * T

  for i in range(MAX_ITER):
    Sbar = PredictState(Sk, oa, od, Sref)
    poa, pod = oa[:], od[:]
    oa, od, ox, oy, oyaw, ov = LinearMPC(Sref, Sbar, Sk, dref)
    dU = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc U change value
    if dU <= DU_TH:
      break
  else:
    #print("Iterative is max iter")
    pass

  return oa, od, ox, oy, oyaw, ov


def LinearMPC(Sref, Sbar, Sk, dref):
  # Sref: reference point, Sbar: operational point, Sk: initial state, dref: ref steer angle

  S = cvxpy.Variable((NS, T + 1))
  U = cvxpy.Variable((NU, T))
    #--- S, U = var1 var2  #--- S, U = var670 var671
    # See Convex Linear Optimization with CVXPY by Dylan Skinner
  cost = 0.0
  constraints = []

  for t in range(T):
    cost += cvxpy.quad_form(U[:, t], Rb)
      #print("#--- cost =", cost)
      #--- cost = QuadForm(var2[0:2, 0], [[0.01 0.  ] [0.   0.01]])
      #print("#--- U.value[:, t] =", U.value[:, t])
      #TypeError: 'NoneType' object is not subscriptable
      #--- S[:, t], U[:, t] = var1[0:4, 0] var2[0:2, 0]

    if t != 0:
      cost += cvxpy.quad_form(Sref[:, t] - S[:, t], Q)

    A, B, C = get_linear_model_matrix(Sbar[2, t], Sbar[3, t], dref[0, t])
    constraints += [S[:, t + 1] == A @ S[:, t] + B @ U[:, t] + C]

    if t < (T - 1):
      cost += cvxpy.quad_form(U[:, t + 1] - U[:, t], R)
      constraints += [cvxpy.abs(U[1, t + 1] - U[1, t]) <= MAX_DSTEER * DT]

  cost += cvxpy.quad_form(Sref[:, T] - S[:, T], Q)

  constraints += [S[:, 0] == Sk]
    #--- [S[:, 0] == Sk], Sk = [Equality(Expression(AFFINE, UNKNOWN, (4,)), Constant(CONSTANT, NONPOSITIVE, (4,)))] [0.0, 0.0, 0.0, -0.083]
  constraints += [S[2, :] <= MAX_SPEED]
  constraints += [S[2, :] >= MIN_SPEED]
  constraints += [cvxpy.abs(U[0, :]) <= MAX_ACCEL]
  constraints += [cvxpy.abs(U[1, :]) <= MAX_STEER]

  prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
  prob.solve(solver=cvxpy.ECOS, verbose=False)
    #print("#--- U.value[:, t] =", U.value[:, t])
    #--- U.value[:, t] = [ 1.0000000e+00 -7.7389417e-22]

  if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
    ox = get_nparray_from_matrix(S.value[0, :])
    oy = get_nparray_from_matrix(S.value[1, :])
    ov = get_nparray_from_matrix(S.value[2, :])
    oyaw = get_nparray_from_matrix(S.value[3, :])
    oa = get_nparray_from_matrix(U.value[0, :])
    odelta = get_nparray_from_matrix(U.value[1, :])
      #--- U.value[0, :], U.value[1, :] = [1. 1. 1. 1. 1.] [-2.17e-21 -1.61e-21 -1.40-21 -1.81-21 -7.73e-22]
  else:
    print("Error: Cannot solve mpc..")
    oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

  return oa, odelta, ox, oy, oyaw, ov


def check_goal(state, goal, tind, nind):
  # check goal
  dx = state.x - goal[0]
  dy = state.y - goal[1]
  d = math.hypot(dx, dy)

  isgoal = (d <= GOAL_DIS)

  if abs(tind - nind) >= 5:
    isgoal = False

  isstop = (abs(state.v) <= STOP_SPEED)

  if isgoal and isstop:
    return True

  return False


def do_simulation(cx, cy, cyaw, ck, cv, dl, S0):
    #cx: course x position list, cy: course y position list, cyaw: course yaw list
    #ck: course curvature list,  cv: speed profile,            dl: course tick [m]
  goal = [cx[-1], cy[-1]]
  state = S0

  time = 0.0
  x = [state.x]
  y = [state.y]
  yaw = [state.yaw]
  v = [state.v]
  t = [0.0]
  d = [0.0]
  a = [0.0]

  target_ind, _ = calc_nearest_index(state, cx, cy, cyaw, 0)
    #--- target_ind = 0
  odelta, oa = None, None
  cyaw = smooth_yaw(cyaw)

  while MAX_TIME >= time:
    Sref, target_ind, dref = RefTrajectory(state, cx, cy, cyaw, ck, cv, dl, target_ind)
    Sk = [state.x, state.y, state.v, state.yaw]  # current state
    oa, odelta, ox, oy, oyaw, ov = NewtonIter(Sref, Sk, dref, oa, odelta)

    di, ai = 0.0, 0.0
    if odelta is not None:
      di, ai = odelta[0], oa[0]
      state = UpdateState(state, ai, di)

    time = time + DT
    x.append(state.x)
    y.append(state.y)
    yaw.append(state.yaw)
    v.append(state.v)
    t.append(time)
    d.append(di)
    a.append(ai)

    if check_goal(state, goal, target_ind, len(cx)):
      print("Goal")
      break

    if show_animation:  # pragma: no cover
      plt.cla()
      # for stopping simulation with the esc key.
      plt.gcf().canvas.mpl_connect('key_release_event',
              lambda event: [exit(0) if event.key == 'escape' else None])
      if ox is not None:
        plt.plot(ox, oy, "xr", label="MPC")
      plt.plot(cx, cy, "-r", label="course")
      plt.plot(x, y, "ob", label="trajectory")
      plt.plot(Sref[0, :], Sref[1, :], "xk", label="Sref")
      plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
      plot_car(state.x, state.y, state.yaw, steer=di)
      plt.axis("equal")
      plt.grid(True)
      plt.title("Time [s]: " + str(round(time, 2))
                + ", Speed [km/h]: " + str(round(state.v * 3.6, 2)))
      plt.pause(0.0001)

  return t, x, y, yaw, v, d, a


def calc_speed_profile(cx, cy, cyaw, target_speed):
  speed_profile = [target_speed] * len(cx)
  direction = 1.0  # forward

  # Set stop point
  for i in range(len(cx) - 1):
    dx = cx[i + 1] - cx[i]
    dy = cy[i + 1] - cy[i]

    move_direction = math.atan2(dy, dx)
    print("#--- i, cx[i], cy[i], move_direction, cyaw[i], move_direction - cyaw[i] =",
          i, round(cx[i], 2), round(cy[i], 2), round(move_direction, 2),
          round(cyaw[i], 2), round(move_direction - cyaw[i], 2))

    if dx != 0.0 and dy != 0.0:
      dangle = abs(pi_2_pi(move_direction - cyaw[i]))
      if dangle >= math.pi / 4.0:
        direction = -1.0
      else:
        direction = 1.0

    if direction != 1.0:
      speed_profile[i] = - target_speed
    else:
      speed_profile[i] = target_speed

  speed_profile[-1] = 0.0

  return speed_profile


def smooth_yaw(yaw):
    # because we may have calc_yaw() => calc_first_derivative() => + or - infinity
    #print("#--- Byaw =", [ round(elem, 2) for elem in yaw ])
    # -0.02, -6.22, -6.13, -6.03, -5.93, ...
  for i in range(len(yaw) - 1):
    dyaw = yaw[i + 1] - yaw[i]

    while dyaw >= math.pi / 2.0:
      yaw[i + 1] -= math.pi * 2.0
      dyaw = yaw[i + 1] - yaw[i]

    while dyaw <= -math.pi / 2.0:
      yaw[i + 1] += math.pi * 2.0
      dyaw = yaw[i + 1] - yaw[i]

    #print("#--- Ayaw =", [ round(elem, 2) for elem in yaw ])
    # -0.02, 0.06, 0.15, 0.25, 0.35, ...
  return yaw


def get_straight_course(dl):
  ax = [0.0, 5.0, 10.0, 20.0, 30.0, 40.0, 50.0]
  ay = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  cx, cy, cyaw, ck, h = cubic_spline_planner.calc_spline_course(ax, ay, dh=dl)
  return cx, cy, cyaw, ck


def get_straight_course2(dl):
  ax = [0.0, -10.0, -20.0, -40.0, -50.0, -60.0, -70.0]
  ay = [0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0]
  cx, cy, cyaw, ck, h = cubic_spline_planner.calc_spline_course(ax, ay, dh=dl)
  return cx, cy, cyaw, ck


def get_straight_course3(dl):
  ax = [0.0, -10.0, -20.0, -40.0, -50.0, -60.0, -70.0]
  ay = [0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0]
  cx, cy, cyaw, ck, h = cubic_spline_planner.calc_spline_course(ax, ay, dh=dl)
  cyaw = [i - math.pi for i in cyaw]
  return cx, cy, cyaw, ck


def get_forward_course(dl):
  ax = [0.0, 60.0, 125.0, 50.0, 75.0, 30.0, -10.0]
  ay = [0.0, 0.0, 50.0, 65.0, 30.0, 50.0, -20.0]
  cx, cy, cyaw, ck, h = cubic_spline_planner.calc_spline_course(ax, ay, dh=dl)
  return cx, cy, cyaw, ck


def get_switch_back_course(dl):
  ax = [0.0, 30.0, 6.0, 20.0, 35.0]
  ay = [0.0, 0.0, 20.0, 35.0, 20.0]
  cx, cy, cyaw, ck, h = cubic_spline_planner.calc_spline_course(ax, ay, dh=dl)
    #--- h = [0.0, 1.0, 2.0, 3.0, ..., 100.0, 101.0, 102.0]
  ax = [35.0, 10.0, 0.0, 0.0]
  ay = [20.0, 30.0, 5.0, 0.0]
  cx2, cy2, cyaw2, ck2, s2 = cubic_spline_planner.calc_spline_course(ax, ay, dh=dl)
    #--- cx2 =   [35.0, 33.96, 32.92, ..., -0.04, -0.03, -0.02]
    #--- cy2 =   [20.0, 20.73, 21.45, ...,  2.81,  1.82,  0.83]
    #--- cyaw2 = [2.53,  2.53,  2.54, ..., -1.57, -1.56, -1.55]
    #--- s2 =    [0.0, 1.0, 2.0, 3.0, ...,        57.0,  58.0]
  cyaw2 = [i - math.pi for i in cyaw2]
  cx.extend(cx2)
  cy.extend(cy2)
  cyaw.extend(cyaw2)
  ck.extend(ck2)

  return cx, cy, cyaw, ck


def main():
  print(__file__ + " start!!")

  dl = 1.0  # course tick
  # cx, cy, cyaw, ck = get_straight_course(dl)
  # cx, cy, cyaw, ck = get_straight_course2(dl)
  # cx, cy, cyaw, ck = get_straight_course3(dl)
  # cx, cy, cyaw, ck = get_forward_course(dl)
  cx, cy, cyaw, ck = get_switch_back_course(dl)

  cv = calc_speed_profile(cx, cy, cyaw, TARGET_SPEED)
    #print("#--- cv =", [ round(elem, 2) for elem in cv ])

  S0 = State(x=cx[0], y=cy[0], v=0.0, yaw=cyaw[0])  # initial state

  t, x, y, yaw, v, d, a = do_simulation(cx, cy, cyaw, ck, cv, dl, S0)

  if show_animation:  # pragma: no cover
    plt.close("all")
    plt.subplots()
    plt.plot(cx, cy, "-r", label="spline")
    plt.plot(x, y, "-g", label="tracking")
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.legend()

    plt.subplots()
    plt.plot(t, v, "-r", label="speed")
    plt.grid(True)
    plt.xlabel("Time [s]")
    plt.ylabel("Speed [kmh]")

    plt.show()


def main2():
  print(__file__ + " start!!")

  dl = 1.0  # course tick
  cx, cy, cyaw, ck = get_straight_course3(dl)

  cv = calc_speed_profile(cx, cy, cyaw, TARGET_SPEED)

  S0 = State(x=cx[0], y=cy[0], v=0.0, yaw=0.0)  # initial state

  t, x, y, yaw, v, d, a = do_simulation(cx, cy, cyaw, ck, cv, dl, S0)

  if show_animation:  # pragma: no cover
    plt.close("all")
    plt.subplots()
    plt.plot(cx, cy, "-r", label="spline")
    plt.plot(x, y, "-g", label="tracking")
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.legend()

    plt.subplots()
    plt.plot(t, v, "-r", label="speed")
    plt.grid(True)
    plt.xlabel("Time [s]")
    plt.ylabel("Speed [kmh]")

    plt.show()


if __name__ == '__main__':
  main()
  # main2()
