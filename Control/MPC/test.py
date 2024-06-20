"""
JLL 230530

(sconsvenv) jinn@Liu:~/openpilot/aMPC$ python test.py
"""
'''
OK: Test 1: For A @ x[:, t] in mpc.py

class Matrix(list):
    def __matmul__(self, B):
        A = self
        return Matrix([[sum(A[i][k] * B[k][j] for k in range(len(B)))
                    for j in range(len(B[0])) ] for i in range(len(A))])
A = Matrix([[1, 2],[3, 4]])
B = Matrix([[5, 6],[7, 8]])
print(A @ B)

OK: T 2: For A @ x[:, t] in mpc.py
import numpy as np
A = np.diag([1, 2])
B = np.diag([3, 4])
print(A @ B)
'''
