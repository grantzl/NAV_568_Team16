import numpy as np
from scipy.linalg import block_diag

a =np.zeros((6,2))
a[1,:] = [1,2]
print(a)

b = np.zeros((6,1))
b[4:6] = np.array([[4], [5]])#
print(b)

A = a, b
print(block_diag(A))