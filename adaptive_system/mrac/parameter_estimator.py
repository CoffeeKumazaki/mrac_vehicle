import numpy as np
from sympy import *
from control.matlab import *

def get_tf_params(sys):
  num, den = tfdata(sys)
  k = num[0][0][0]
  z = num[0][0]/k
  r = den[0][0]
  return k, z, r

def solve_opt_parameters(kp: float, zp: list, rp: list, km: float, zm: list, rm: list, dim: int, lambda0: list):
  s=symbols('s')

  def param_poly(dof, symbol, param_prefix):
    a = []
    for i in range(dof):
      t = symbols(param_prefix+str(i))
      a.append(t)
      
    return Poly(a, symbol), a

  zp = Poly(zp, s)
  zm = Poly(zm, s)
  rp = Poly(rp, s)
  rm = Poly(rm, s)
  lbd0 = Poly(lambda0, s)

  lbd = lbd0*zm

  theta1, t1 = param_poly(dim, s, 't1')
  theta2, t2 = param_poly(dim, s, 't2')
  theta3, t3 = param_poly(1, s, 't3')

  lhs = Poly(theta1 * rp + kp*(theta2 + theta3*lbd)*zp)  
  rhs = Poly(lbd*rp - zp*lbd0*rm)

  lhs_coef = lhs.all_coeffs()
  rhs_coef = rhs.all_coeffs()

  eqs = []
  for l, r in zip(lhs_coef, rhs_coef):
    eqt = Eq(l, r)
    eqs.append(eqt)


  print("number of equations ", len(rhs.all_coeffs()))
  print("number of parameters", len(t1+t2+t3))

  sol = solve(eqs, t1 + t2 + t3)

  res = {}
  for k in sol.keys():
    res[str(k)] = float(sol[k])

  return res

def estimate_theta(plant_tf, reference_tf, omega_dim, lambda0_tf_coeffs):
  kp, zp, rp = get_tf_params(plant_tf)
  km, zm, rm = get_tf_params(reference_tf)
  theta = solve_opt_parameters(kp, zp, rp, km, zm, rm, omega_dim, lambda0_tf_coeffs)

  y_dim = reference_tf.noutputs;
  etheta = np.zeros((1+ y_dim + 2*omega_dim, 1))

  id = 0
  for i in range(omega_dim):
    etheta[id][0] = theta['t1'+str(i)]
    id += 1

  for i in range(omega_dim):
    etheta[id][0] = theta['t2'+str(i)]
    id += 1

  for i in range(y_dim):
    etheta[id][0] = theta['t3'+str(i)]
    id += 1

  etheta[id][0] = km/kp


  return etheta



def current_tf(plant_tf, theta, omega_dim, lambda_tf_coeffs):
  '''
  Iannou & Sun
  Chapter 6.3 
  P.339
  '''
  s = symbols('s')

  kp, zp_coef, rp_coef = get_tf_params(plant_tf)

  zp = Poly(zp_coef, s)
  rp = Poly(rp_coef, s)

  theta_1_alpha = Poly(theta[0:omega_dim], s)
  theta_2_alpha = Poly(theta[omega_dim:2*omega_dim], s)

  lbd = Poly(lambda_tf_coeffs, s)

  num = theta[-1]*kp*zp*lbd
  den = ( (lbd - theta_1_alpha)*rp - kp*zp*(theta_2_alpha + theta[-2]*lbd) )

  return np.array(num.all_coeffs()).astype(float) , np.array(den.all_coeffs()).astype(float) 
