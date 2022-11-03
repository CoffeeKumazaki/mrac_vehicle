from sympy import *
from control import *
from control.matlab import *
from vehicle_params import *
from vehicle_dynamics import *
from mrac.controller_designer import *
from mrac.parameter_estimator import get_tf_params

def solve_opt_parameters(kp: float, zp: list, rp: list, km: float, zm: list, rm: list, dim: int, lambda0: list):
  s = symbols('s')

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

  lhs = Poly(-lbd*rp + theta1 * rp + kp*(theta2 + theta3*lbd)*zp)  
  rhs = Poly(-zp*lbd0*rm)

  pprint(lhs)
  exit()
  # print("")
  # print(lhs.args[0])
  pprint(factor(lhs, extension=solveset(lhs.args[0], s)))

  sol = solve(lhs, s)
  # pprint(sol)


vx = 15

## Reference model
modelParam = VehicleParam()
A, B, C, D = linear_vehicle_model(modelParam, vx)
mss = ss(A, B, C, D)

lbd0 = [1, 1]
sv_dim, L, l = designed_state_space_eq(mss, lbd0)

tf_lambda = tf(ss(L, l, np.eye(sv_dim), 0))
nlam, dlam = tfdata(tf_lambda)

## plant LTI model
plantParam = VehicleParam()
plantParam.mass = 1727.0
plantParam.cf = 94000.0
plantParam.cr = 94000.0
plantParam.lf = 1.17
plantParam.lr = 1.42
plantParam.Iz = 2867.0
'''
plantParam.mass = 5500.0
plantParam.cf = 50000.0
plantParam.cr = 130000.0
plantParam.lf = 2.5
plantParam.lr = 1.5
plantParam.Iz = 20600.0
'''


A, B, C, D = linear_vehicle_model(plantParam, vx)
pss = ss(A, B, C, D)
pnum, pden = tfdata(pss)

kp, zp, rp = get_tf_params(pss)
km, zm, rm = get_tf_params(mss)


solve_opt_parameters(kp, zp, rp, km, zm, rm, sv_dim, lbd0)