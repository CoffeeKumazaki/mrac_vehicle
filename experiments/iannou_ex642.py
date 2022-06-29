from math import sin
import tqdm
import traceback
import numpy as np
from control.matlab import *
from scipy.spatial import KDTree

from linear_system import *
from vehicle_params import *
from vehicle_dynamics import *
from vehicle import *
from mrac.controller import *
from mrac.parameter_estimator import *
from mrac.controller_designer import *

def sim(simT, dt, plant, controller):
  
  Ts = np.arange(0,simT, dt)

  r = np.array([[0.0]])
  u = np.array([[0.0]])

  for t in tqdm.tqdm(Ts):

    yp = plant.observe()
    controller.update(dt, yp, r, u)

    r = reference_input(t)
    u = adaptive_ctrl.calc_u(yp, r)
    # xp = plant.x
    # plant.update(dt, [-xp[5]*xp[4]*plant.vehicle_param.mass, u])
    plant.update(dt, u)

    yr = controller.ref.observe()
    yp = plant.observe()

    e = yp[0]-yr[0]
    theta = controller.theta.T[0]
    yield t, yp[0], yr[0], e, theta, u[0], r[0], plant.state.T[0]


def data_header(plant_param, reference_param):
  return f'''
    Plant Parameters 
      type : {plant_type}
    {plant_param}
     
    MRAC parameter
     gain : {adaptive_gain}
     lbd0 : {lbd0}
     vx   : {vx}
     umax : {umax} 
     umin : {umin} 
    Reference Parameters
    {reference_param}

    use initial guess : {use_initial_guess}

    analytical adaptive param:
    {estTheta} 
  '''


lbd0 = [1, 2]
plant_type = "example642"
adaptive_gain = np.array([2, 4, 0.8, 1]) * 50
umax = None
umin = None
simT = 180
use_initial_guess = False

def reference_input(t):
  r = np.array([[float(3.0*sin(4.9*t) + 0.5*sin(0.7*t))]])
  return r
  # return np.array([[0.0]])

filename_prefix = plant_type + "_gain_1"

## Reference model
modelParam = VehicleParam()
mss = tf([5], [1, 5, 25])
ref = LinearSystem(mss, 0, "ref")

sv_dim, L, l = designed_state_space_eq(mss, lbd0)
w1 = LinearSystem(ss(L, l, np.identity(sv_dim), 0), 0, "w1")
w2 = LinearSystem(ss(L, l, np.identity(sv_dim), 0), 0, "w2")

phi_dim = sv_dim*2 + ref.sys.ninputs + ref.sys.noutputs
Lp = -np.identity(phi_dim)*1
lp = np.identity(phi_dim)*1
phi = LinearSystem(ss(Lp, lp, np.identity(phi_dim), 0), 0, 'phi')

adaptive_ctrl = AdaptiveControllerN2(ref, w1, w2, phi, adaptive_gain, umin=umin, umax=umax)

# set estimated param

## plant LTI model
pss = tf([1], [1, 3, -15])

print(tf(pss))
print(tf(mss))
print(adaptive_ctrl)

estTheta = estimate_theta(pss, mss, sv_dim, lbd0)
print(estTheta)

### give initial parameters
if (use_initial_guess):
  adaptive_ctrl.theta = estTheta

# adaptive_ctrl.theta[0] = 0
# adaptive_ctrl.theta[1] = -20
# adaptive_ctrl.theta[2] = -30
# adaptive_ctrl.theta[3] = 2


plant = LinearSystem(pss, 0)

res = sim(simT, 0.01, plant, adaptive_ctrl)


ts = []
us = []
yrs = []
yps = []
es = []
rs = []
thetas = []
pxs = []
while (1):
  try:
    t, yp, yr, e, theta, u, r, px = next(res)

    ts.append(t)
    yps.append(yp)
    yrs.append(yr)
    es.append(e)
    us.append(u)
    rs.append(r)
    thetas.append(theta)
    pxs.append(px)
  
  except Exception as e:
    print("finish", e)
    print(traceback.format_exc())
    break


foldername = "with_init_param" if use_initial_guess else "no_init_param"

print("## results")
print("../data/output/" + foldername +"/" + filename_prefix + ".csv")
np.savetxt("../data/output/" + foldername +"/" + filename_prefix + ".csv", np.array(pxs), delimiter=" ")
np.savetxt("../data/output/" + foldername +"/" + filename_prefix + "_theta.csv", np.array(thetas), delimiter=" ")
np.savetxt("../data/output/" + foldername +"/" + filename_prefix + "_u.csv", np.array(us), delimiter=" ")
np.savetxt("../data/output/" + foldername +"/" + filename_prefix + "_yp.csv", yps, delimiter=" ")
np.savetxt("../data/output/" + foldername +"/" + filename_prefix + "_yr.csv", yrs, delimiter=" ")
np.savetxt("../data/output/" + foldername +"/" + filename_prefix + "_e.csv", es, delimiter=" ")