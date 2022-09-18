from math import *
import tqdm
import traceback
import numpy as np
from control.matlab import *
from scipy.spatial import KDTree
from pykalman import KalmanFilter

from linear_system import *
from vehicle_params import *
from vehicle_dynamics import *
from vehicle import *
from mrac.controller import *
from mrac.parameter_estimator import *
from mrac.controller_designer import *

def sim(simT, dt, plant, controller, kf):
  
  Ts = np.arange(0,simT, dt)

  r = np.array([[0.0]])
  u = np.array([[0.0]])
  cov = kf.initial_state_covariance
  state = kf.initial_state_mean
  tofs = np.array(kf.transition_offsets).T
  for t in tqdm.tqdm(Ts):

    yp = plant.observe()
    ypt = plant.observe(False)
    state, cov = kf.filter_update(state, cov, observation=np.array(yp[0]), transition_offset=tofs.dot(u).T[0])
    yf = np.array([[state.dot(kf.observation_matrices)]])
    controller.update(dt, yf, r, u)
    yr = controller.ref.observe()
    e = yp[0]-yr[0]

    r = reference_input(t)
    u = adaptive_ctrl.calc_u(yf, r)
    xp = plant.x
    plant.update(dt, [-xp[5]*xp[4]*plant.vehicle_param.mass, u])
    # plant.update(dt, u)

    y = np.hstack([yp[0], ypt[0], yf[0], yr[0]])
    # yp = plant.observe(False)

    theta = controller.theta.T[0]
    yield t, yp[0], yr[0], e, theta, u[0], r[0], plant.x, controller.k1[0], controller.e1[0], controller.e2[0], y


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


plantParam = VehicleParam()

## Parameter settings
vx = 20
lbd0 = [1, 1]
plant_type = "vehicle"
adaptive_gain = 1.0
umax = 0.4
umin = -umax
simT = 180
dt = 0.01
robust = "deadzone" # nothing of deadzone
rbParam = 1.0
noise_std = 0.2

scenario = "straight" # tomei or straight

def reference_input(t):
  r = np.array([[float(0.1*sin(1.0*t))]])
  return r
  # return np.array([[0.0]])

road_file = ""
if scenario == "tomei":
  road_file = "../data/tomei_sp2.txt"
else:
  road_file = "../data/straight.txt"

road = np.loadtxt(road_file, delimiter=" ", dtype=np.float32)
tree = KDTree(road[:, :2])

if robust == "nothing":
  filename_prefix = plant_type + "_vx_" + str(int(vx)) + "_gain_" + str(int(adaptive_gain)) + "_" + scenario + "_" + "u04_01sin1" + "_noise_" + str(noise_std) + "_mass120"
else:
  filename_prefix = plant_type + "_" + robust + "_" + str(rbParam) + "_vx_" + str(int(vx)) + "_gain_" + str(int(adaptive_gain)) + "_" + scenario + "_" "u04_01sin1" + "_noise_" + str(noise_std) + "_mass120"

use_initial_guess = True

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

kf = KalmanFilter(transition_matrices = np.identity(4) + np.array(A)*dt,
                  transition_offsets = np.array(B).T*dt,
                  observation_matrices = C,
                  transition_covariance = np.diag([1e-4, 1e-4, 1e-4, 0.1]),
                  observation_covariance = [10.0],
                  initial_state_mean = np.zeros(4),
                  initial_state_covariance = np.diag([1e-9, 1e-9, 1e-9, noise_std*noise_std]),
                  em_vars = 'all'
                  )

sv_dim, L, l = designed_state_space_eq(mss, lbd0)
w1 = LinearSystem(ss(L, l, np.identity(sv_dim), 0), 0, "w1")
w2 = LinearSystem(ss(L, l, np.identity(sv_dim), 0), 0, "w2")

adaptive_ctrl = ConstraintAdaptiveControllerN2(ref, w1, w2, adaptive_gain, umin=umin, umax=umax, robust=robust, rbParam=rbParam)

# set estimated param

## plant LTI model
A, B, C, D = linear_vehicle_model(plantParam, vx)
pss = ss(A, B, C, D)

print(tf(mss))
print(tf(pss))

estTheta = estimate_theta(pss, mss, sv_dim, lbd0)
print(estTheta)
header = data_header(plantParam, modelParam)

### give initial parameters
if (use_initial_guess):
  estTheta = estimate_theta(pss, mss, sv_dim, lbd0)
else:
  tempParam = VehicleParam()
  A, B, C, D = linear_vehicle_model(tempParam, vx)
  tss = ss(A, B, C, D)

  estTheta = estimate_theta(tss, mss, sv_dim, lbd0)
  print(estTheta)

adaptive_ctrl.theta = estTheta

# adaptive_ctrl.theta = estTheta*1.1

plantParam.mass = plantParam.mass*1.2
A, B, C, D = linear_vehicle_model(plantParam, vx)
ptss = ss(A, B, C, D)
print(tf(ptss))
plant = Vehicle(plantParam, plantInit, road, noise_std)

res = sim(simT, dt, plant, adaptive_ctrl, kf=kf)


ts = []
us = []
yrs = []
yps = []
ypts = []
es = []
rs = []
thetas = []
pxs = []
k1s = []
e1s = []
e2s = []
while (1):
  try:
    t, yp, yr, e, theta, u, r, px, k1, e1, e2, ypt = next(res)

    ts.append(t)
    yps.append(yp)
    yrs.append(yr)
    ypts.append(ypt)
    es.append(e)
    us.append(u)
    rs.append(r)
    thetas.append(theta)
    pxs.append(px)
    k1s.append(k1)
    e1s.append(e1)
    e2s.append(e2)
  
  except Exception as e:
    print("finish", e)
    print(traceback.format_exc())
    break

foldername = "with_init_param" if use_initial_guess else "no_init_param"

print("## results")
print("../data/output/" + foldername +"/" + filename_prefix + ".csv")
np.savetxt("../data/output/" + foldername +"/" + filename_prefix + ".csv", np.array(pxs), delimiter=" ", header=header)
np.savetxt("../data/output/" + foldername +"/" + filename_prefix + "_theta.csv", np.array(thetas), delimiter=" ", header=header)
np.savetxt("../data/output/" + foldername +"/" + filename_prefix + "_u.csv", np.array(us), delimiter=" ", header=header)
np.savetxt("../data/output/" + foldername +"/" + filename_prefix + "_yp.csv", yps, delimiter=" ", header=header)
np.savetxt("../data/output/" + foldername +"/" + filename_prefix + "_yr.csv", yrs, delimiter=" ", header=header)
np.savetxt("../data/output/" + foldername +"/" + filename_prefix + "_ypt.csv", ypts, delimiter=" ", header=header)
np.savetxt("../data/output/" + foldername +"/" + filename_prefix + "_r.csv", rs, delimiter=" ", header=header)
np.savetxt("../data/output/" + foldername +"/" + filename_prefix + "_e.csv", es, delimiter=" ", header=header)
np.savetxt("../data/output/" + foldername +"/" + filename_prefix + "_k1.csv", np.array(k1s), delimiter=" ", header=header)
np.savetxt("../data/output/" + foldername +"/" + filename_prefix + "_e1.csv", np.array(e1s), delimiter=" ", header=header)
np.savetxt("../data/output/" + foldername +"/" + filename_prefix + "_e2.csv", np.array(e2s), delimiter=" ", header=header)