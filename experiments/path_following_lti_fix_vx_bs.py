from math import sin
import tqdm
import traceback
import numpy as np
from control.matlab import *
from scipy.spatial import KDTree

from linear_system import *
from vehicle_params import *
from vehicle_dynamics import *
from vehicle_LTI import *
from mrac.controller_designer import *
from backstepping.controller import BacksteppingController


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
lbd0 = [1, 1]
plant_type = "vehicle"
adaptive_gain = 1.0
pgamma = 0.01
c1 = 1.0
c2 = 2.0
umax = 100000.0
umin = -umax
simT = 180

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

sv_dim, L, l = designed_state_space_eq(mss, lbd0)
w1 = LinearSystem(ss(L, l, np.identity(sv_dim), 0), 0, "w1")
w2 = LinearSystem(ss(L, l, np.identity(sv_dim), 0), 0, "w2")

## Backstepping controller (w1 filters u, w2 filters yp - same as MRAC)
bs_ctrl = BacksteppingController(ref, w1, w2, adaptive_gain, pgamma, c1, c2, umin, umax)

## Plant LTI model
A, B, C, D = linear_vehicle_model(plantParam, vx)
pss = ss(A, B, C, D)

print("Plant TF:")
print(tf(pss))
print("Reference TF:")
print(tf(mss))

plant = LinearSystem(pss, plantInit)

res = sim(simT, 0.01, plant, bs_ctrl)

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


header = f'''
  Plant Parameters
    type : {plant_type}
  {plantParam}

  Backstepping parameters
   gain : {adaptive_gain}
   pgamma : {pgamma}
   c1 : {c1}
   c2 : {c2}
   lbd0 : {lbd0}
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
