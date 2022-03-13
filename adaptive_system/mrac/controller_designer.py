import numpy as np
from sympy import *

from mrac.parameter_estimator import get_tf_params

def designed_state_space_eq(ref, lambda0):

  _, zm_coeffs, _ = get_tf_params(ref)

  s = symbols('s')
  lbd0 = Poly(lambda0, s)
  zm = Poly(zm_coeffs, s)
  lbd = lbd0*zm

  n = degree(lbd, s)
  coeffs = -np.array(lbd.all_coeffs()).astype(float)[1:]

  L = np.zeros((n, n))

  L[0] = coeffs
  for i in range(n-1):
    L[i+1, i] = 1.0

  l = np.zeros((n, 1))
  l[0, 0] = 1.0

  return n, L, l