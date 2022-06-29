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
    xp = plant.x
    plant.update(dt, [-xp[5]*xp[4]*plant.vehicle_param.mass, u])
    # plant.update(dt, u)

    yr = controller.ref.observe()
    yp = plant.observe()

    e = yp[0]-yr[0]
    theta = controller.theta.T[0]
    yield t, yp[0], yr[0], e, theta, u[0], r[0], plant.x


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


road = np.loadtxt("../data/tomei_sp2.txt", delimiter=" ", dtype=np.float32)
tree = KDTree(road[:, :2])

plantParam = VehicleParam()
'''
plantParam.mass = 1727.0
plantParam.cf = 94000.0
plantParam.cr = 94000.0
plantParam.lf = 1.17
plantParam.lr = 1.42
plantParam.Iz = 2867.0

plantParam.mass = 5500.0
plantParam.cf = 50000.0
plantParam.cr = 130000.0
plantParam.lf = 2.5
plantParam.lr = 1.5
plantParam.Iz = 20600.0
'''


## Parameter settings
vx = 20
lbd0 = [1, 1]
plant_type = "vehicle"
adaptive_gain = 100.0
umax = 0.4
umin = -0.4
simT = 180

def reference_input(t):
  r = np.array([[float(0.01*sin(1.0*t))]])
  # return r
  return np.array([[0.0]])

filename_prefix = plant_type + "_vx_20_gain_100_tomei_u04"
use_initial_guess = False

## processing
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
else:
  plant_type = "prius"


## Initial parameter
plantInit = [road[0,0], road[0,1], road[0,2], vx, 0, 0, 0]

## Reference model
modelParam = VehicleParam()
A, B, C, D = linear_vehicle_model_fb(modelParam, vx, -2.0, -0.05)
mss = ss(A, B, C, D)
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
A, B, C, D = linear_vehicle_model(plantParam, vx)
pss = ss(A, B, C, D)

print(tf(pss))
print(tf(mss))

estTheta = estimate_theta(pss, mss, sv_dim, lbd0)
print(estTheta)
adaptive_ctrl.theta[-2] = -0.05
adaptive_ctrl.theta[-1] = 1.0

### give initial parameters
if (use_initial_guess):
  adaptive_ctrl.theta = estTheta

plant = Vehicle(plantParam, plantInit, road)

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


header = data_header(plantParam, modelParam)

foldername = "with_init_param" if use_initial_guess else "no_init_param"

print("## results")
print("../data/output/" + foldername +"/" + filename_prefix + ".csv")
np.savetxt("../data/output/" + foldername +"/" + filename_prefix + ".csv", np.array(pxs), delimiter=" ", header=header)
np.savetxt("../data/output/" + foldername +"/" + filename_prefix + "_theta.csv", np.array(thetas), delimiter=" ", header=header)
np.savetxt("../data/output/" + foldername +"/" + filename_prefix + "_u.csv", np.array(us), delimiter=" ", header=header)
np.savetxt("../data/output/" + foldername +"/" + filename_prefix + "_yp.csv", yps, delimiter=" ", header=header)
np.savetxt("../data/output/" + foldername +"/" + filename_prefix + "_yr.csv", yrs, delimiter=" ", header=header)
np.savetxt("../data/output/" + foldername +"/" + filename_prefix + "_e.csv", es, delimiter=" ", header=header)