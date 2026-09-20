from math import sin
import tqdm
import numpy as np
from control.matlab import *

from linear_system import *
from vehicle_params import *
from vehicle_dynamics import *
from vehicle_LTI import *
from backstepping.controller import AdaptiveBacksteppingController


def sim(simT, dt, plant, controller):

  Ts = np.arange(0, simT, dt)

  for t in tqdm.tqdm(Ts):

    r = reference_input(t)

    yp = plant.observe()

    u = controller.calc_u(yp, r)
    plant.update(dt, u)

    controller.update(dt, yp, r, u)

    yr = controller.ref.observe()
    yp = plant.observe()

    e = yp[0] - yr[0]
    theta = controller.theta.T[0]
    yield t, yp[0], yr[0], e, theta, u[0], r[0], plant.state.T[0]


## Parameter settings
vx = 10
plant_type = "vehicle"
adaptive_gain = 1.0
c1 = 2.0
c2 = 2.0
d1 = 0.05
d2 = 0.05
filter_poles = -5.0
umax = 100000.0
umin = -umax
simT = 180
dt = 0.01

def reference_input(t):
  r = np.array([[float(0.01 * sin(0.1 * t))]])
  return r

filename_prefix = plant_type + "_lti_vx_10_bs"

## processing
plantParam = VehicleParam()
if plant_type == "vehicle":
  plantParam.mass = 1727.0
  plantParam.cf = 94000.0
  plantParam.cr = 94000.0
  plantParam.lf = 1.17
  plantParam.lr = 1.42
  plantParam.Iz = 2867.0
elif plant_type == "truck":
  plantParam.mass = 5500.0
  plantParam.cf = 50000.0
  plantParam.cr = 130000.0
  plantParam.lf = 2.5
  plantParam.lr = 1.5
  plantParam.Iz = 20600.0

## Initial parameter
plantInit = [0, 0, 0, 0]

## Reference model
modelParam = VehicleParam()
A, B, C, D = linear_vehicle_model_fb(modelParam, vx, -2.0, -0.05)
mss = ss(A, B, C, D)
ref = LinearSystem(mss, 0, "ref")

## Backstepping controller: only the reference model, plant order/relative
## degree, and the sign of the high-frequency gain are assumed known.
## rho_hat0 is initialized from the nominal (reference) vehicle: 1/b_m ~ mass/cf.
bs_ctrl = AdaptiveBacksteppingController(
  ref, n=4, filter_poles=filter_poles, gamma=adaptive_gain,
  c1=c1, c2=c2, d1=d1, d2=d2, sgn_bm=1.0,
  rho_hat0=modelParam.mass / modelParam.cf,
  umin=umin, umax=umax)

## Plant LTI model
A, B, C, D = linear_vehicle_model(plantParam, vx)
pss = ss(A, B, C, D)

print("Plant TF:")
print(tf(pss))
print("Reference TF:")
print(tf(mss))

plant = LinearSystem(pss, plantInit)

ts = []
us = []
yrs = []
yps = []
es = []
rs = []
thetas = []
pxs = []
for t, yp, yr, e, theta, u, r, px in sim(simT, dt, plant, bs_ctrl):
  ts.append(t)
  yps.append(yp)
  yrs.append(yr)
  es.append(e)
  us.append(u)
  rs.append(r)
  thetas.append(theta)
  pxs.append(px)


header = f'''
  Plant Parameters
    type : {plant_type}
  {plantParam}

  Adaptive backstepping parameters (K-filters + tuning functions)
   gain : {adaptive_gain}
   c1 : {c1}
   c2 : {c2}
   d1 : {d1}
   d2 : {d2}
   filter_poles : {filter_poles}
   vx   : {vx}
   umax : {umax}
   umin : {umin}
  Reference Parameters
  {modelParam}
'''

foldername = "no_init_param"

print("## results")
print("../data/output/" + foldername + "/" + filename_prefix + ".csv")
np.savetxt("../data/output/" + foldername + "/" + filename_prefix + ".csv", np.array(pxs), delimiter=" ", header=header)
np.savetxt("../data/output/" + foldername + "/" + filename_prefix + "_theta.csv", np.array(thetas), delimiter=" ", header=header)
np.savetxt("../data/output/" + foldername + "/" + filename_prefix + "_u.csv", np.array(us), delimiter=" ", header=header)
np.savetxt("../data/output/" + foldername + "/" + filename_prefix + "_yp.csv", yps, delimiter=" ", header=header)
np.savetxt("../data/output/" + foldername + "/" + filename_prefix + "_yr.csv", yrs, delimiter=" ", header=header)
np.savetxt("../data/output/" + foldername + "/" + filename_prefix + "_e.csv", es, delimiter=" ", header=header)
np.savetxt("../data/output/" + foldername + "/" + filename_prefix + "_r.csv", rs, delimiter=" ", header=header)
