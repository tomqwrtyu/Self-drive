'''
by A. D. Straw, 2006.7.24 - 2018.7.6; modified by JLL, 2022.6.23, 2023.7.5
from Kalman filter in Python of the example in
  [Wel06] G. Welch and G. Bishop, An Introduction to the Kalman Filter, 2006
  https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf
  Ref: A. Becker, Kalman Filter Tutorial: https://www.kalmanfilter.net/kalman1d.html
       White Noise Time Series with Python
       Difference Between Covariance and Correlation: A Definitive Guide
       OP: https://github.com/commaai/rednose/
We adopt the notations in
  [Liu23a] J.-L. Liu, "AI-Based Kalman Filter Control for Autonomous Driving", 2023
for 230708, /selfdrive/controls/radard.py
(sconsvenv) jinn@Liu:~/openpilot/aKalman$ python kalman1.py
'''

import numpy as np
import matplotlib.pyplot as plt

n_iter = 50  # number of iterations
sz = n_iter  # size of array

x = -0.37727  # true value
z = np.random.normal(x, 0.1, size=sz)  # measurements (normal about x, sigma=0.1)
V = 1e-5  # process noise covariance
W = 1e-2  # measurement noise covariance, change to see effect

xhat = np.zeros(sz)      # a posteri estimate of x
E = np.zeros(sz)         # a posteri error covariance
xhatminus = np.zeros(sz) # a priori estimate of x
Eminus = np.zeros(sz)    # a priori error covariance
K = np.zeros(sz)         # gain

xhat[0] = 0.0  # intial guess
E[0] = 1.0     # intial guess

for k in range(1, n_iter):
      # time update
    xhatminus[k] = xhat[k-1]  # (1.9): A = 1, B = 0
    Eminus[k] = E[k-1] + V

      # measurement update
    K[k] = Eminus[k]/(Eminus[k] + W)  # (1.11): M = 1
    xhat[k] = xhatminus[k] + K[k]*(z[k] - xhatminus[k])
    E[k] = (1-K[k])*Eminus[k]

plt.rcParams['figure.figsize'] = (10, 8)
plt.figure()
plt.subplot(1, 2, 1)
plt.plot(z,'k+',label='noisy measurements')
plt.plot(xhat,'b-',label='a posteri estimate')
plt.axhline(x,color='g',label='true value')
plt.legend()
plt.title('State Estimation', fontweight='bold')
plt.xlabel('Iteration')
plt.ylabel('Voltage')

plt.subplot(1, 2, 2)
'''valid_iter = range(1,n_iter) # Eminus not valid at step 0
  plt.plot(valid_iter,Eminus[valid_iter],label='a priori error estimate')
  plt.title('Estimated $\it{\mathbf{a \ priori}}$ error vs. iteration step', fontweight='bold')
  plt.xlabel('Iteration')
  plt.ylabel('$(Voltage)^2$')
  plt.setp(plt.gca(),'ylim',[0,.01])'''
valid_iter = range(1, n_iter)
plt.plot(K[valid_iter], 'k+')
plt.text(10, 0.96, '  K[1] = '+str(round(K[1], 3)))
plt.text(10, 0.92, 'K[49] = '+str(round(K[49], 3)))
plt.text(10, 0.88, 'E[49] = '+str(round(E[49], 5)))
plt.text(10, 0.84, 'V = '+str(V)+', W = '+str(W))
plt.text(10, 0.8, 'K = (E+V)/(E+V+W) = '+str(round((E[49]+V)/(E[49]+V+W), 3)))
plt.title('Kalman Gain', fontweight='bold')
plt.xlabel('Iteration')
plt.ylabel('Gain')
plt.show()
