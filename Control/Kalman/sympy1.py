# JLL, 2022.7.1
# sympy tutorials: https://docs.sympy.org/latest/index.html
# for openpilot/rednose_repo/examples/kinematic_kf.py
# (OP082) jinn@Liu:~/openpilot/rednose_repo/examples$ python sympy1.py

import sympy as sp

a = ("a", "b", "c", "d", "e", "f", "g", "h")
x = slice(2)  # slice(start, end, step), (optional, end, optional)
print('#--- a[x] =', a[x])
  #--- a[x] = ('a', 'b')

x = slice(1, 2)
print('#--- a[x] =', a[x])

M = sp.Matrix(2, 2, [1, 2, 3, 4])
print('#--- M =', M)
print('#--- sp.det(M) =', sp.det(M))

class States():
  POSITION = slice(0, 1)
  VELOCITY = slice(1, 2)
  #--- POSITION, VELOCITY = slice(0, 1, None) slice(1, 2, None)

state_sym = sp.MatrixSymbol('state', 4, 3)
print('#--- state_sym =', state_sym)
state = sp.Matrix(state_sym)
print('#--- state =', state)
print('#--- state[2,1] =', state[2,1])
print('#--- state[2,:][0,2] =', state[2,:][0,2])
print('#--- state[2,:][2] =', state[2,:][2])
print('#--- state[2,:][-1] =', state[2,:][-1])
print('#--- state[2,:][0:2] =', state[2,:][0:2])
print('#--- state[2,:][0,:] =', state[2,:][0,:])

position = state[States.POSITION, :][0,:]
velocity = state[States.VELOCITY, :][0,:]
print('#--- position, velocity =', position, velocity)
