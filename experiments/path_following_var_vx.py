import tqdm
import traceback
import numpy as np
from control.matlab import *
from scipy.spatial import KDTree
import cvxopt
from cvxopt import matrix

from linear_system import *
from vehicle_params import *
from vehicle_dynamics import *
from vehicle import *
from mrac.controller import *
from mrac.parameter_estimator import *
from mrac.controller_designer import *

cvxopt.solvers.options['show_progress'] = False

def sim(simT, dt, plant, controller):
  
  Ts = np.arange(0,simT, dt)

  fx_ref = plant.vehicle_param.mass * 9.8 * 0.2
  fx_max = plant.vehicle_param.mass * 9.8 * 0.4
  fx_min = -plant.vehicle_param.mass * 9.8 * 0.4

  for t in tqdm.tqdm(Ts):

    r = np.array([[0.0]])

    yp = plant.observe()

    ### no steering
    # yp = np.zeros_like(yp)

    u = adaptive_ctrl.calc_u(yp, r)
    # u = np.clip(u, -1.0, 1.0)
    xp = plant.x
    tvx = target_vx if (t > 100.0) else 5.0
    fxref = fx_ref if (t > 100.0) else 0.0
    current_vx = plant.x[3] # sqrt(plant.x[3]*plant.x[3] + plant.x[4]*plant.x[4])
    # fx = calc_acc(plant.vehicle_param.mass, current_vx, tvx, fxref, fx_max, fx_min)
    
    fx = -0.01*(current_vx - tvx)*plant.vehicle_param.mass * 9.8
    fx = np.clip(fx, fx_min, fx_max)
    plant.update(dt, [fx, u])
    #plant.update(dt, u)

    controller.update(dt, yp, r)

    yr = controller.ref.observe()

    e = yp[0]-yr[0]
    theta = controller.theta.T[0]
    yield t, yp[0], yr[0], e, theta, u[0], r[0], xp, fx

def calc_acc(mass, current_velocity, target_velocity, uref, umax, umin, th=1.8):

  # u, dv, dp
  P=matrix(np.diag([2.0, 100000.0, 100000.0]))
  q=matrix(np.array([-2.0*uref, 0.0, 0.0]))

  dv = current_velocity - target_velocity

  G=matrix(np.array([
                    [dv, -1.0, 0.0], 
                    [1.0, 0.0, -1.0], 
                    [-1.0, 0.0, 1.0],
                    ]).astype(np.double))
  h=matrix(np.array([
                    0.0,
                    umax, 
                    umin]))
  # print("P\n", P)
  # print(q)
  # print(G)
  # print(h)
  sol=cvxopt.solvers.qp(P,q,G,h)

  # print(sol)
  # print(sol["x"])
  # print(sol["primal objective"])
  return sol["x"][0]

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

road = np.loadtxt("../data/path.txt", delimiter=" ", dtype=np.float32)
tree = KDTree(road[:, :2])

plantParam = VehicleParam()

## Parameter settings
vx = 5.0
target_vx = 10.0

ref_vx = 20.0
lbd0 = [1, 1]
plant_type = "vehicle"
adaptive_gain = 100.0
umax = 0.1
umin = -0.1

filename_prefix = plant_type + "_vx_05to10_gain_100"
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
A, B, C, D = linear_vehicle_model_fb(modelParam, ref_vx, -2.0, -0.05)
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

print("-- the transfer function of plant")
print(tf(pss))

print("-- the transfer function of reference model")
print(tf(mss))

estTheta = estimate_theta(pss, mss, 3, [1,1])
print(estTheta)
adaptive_ctrl.theta[-2] = -0.05

if (use_initial_guess):
  adaptive_ctrl.theta = estTheta

plant = Vehicle(plantParam, plantInit, road)

res = sim(180, 0.01, plant, adaptive_ctrl)


ts = []
us = []
fxs = []
yrs = []
yps = []
es = []
rs = []
thetas = []
pxs = []
while (1):
  try:
    t, yp, yr, e, theta, u, r, px, fx = next(res)

    ts.append(t)
    yps.append(yp)
    yrs.append(yr)
    es.append(e)
    us.append(u)
    rs.append(r)
    thetas.append(theta)
    pxs.append(px)
    fxs.append(fx)
  
  except Exception as e:
    print(e)
    print(traceback.format_exc())
    print("finish")
    break

header = data_header(plantParam, modelParam)
foldername = "with_init_param" if use_initial_guess else "no_init_param"

print("## results")
print("../data/output/" + foldername +"/" + filename_prefix + ".csv")
np.savetxt("../data/output/" + foldername + "/" + filename_prefix + "_theta.csv", np.array(thetas), delimiter=" ", header=header)
np.savetxt("../data/output/" + foldername + "/" + filename_prefix + "_u.csv", np.array(us), delimiter=" ", header=header)
np.savetxt("../data/output/" + foldername + "/" + filename_prefix + "_fx.csv", np.array(fxs), delimiter=" ", header=header)
np.savetxt("../data/output/" + foldername + "/" + filename_prefix + ".csv", np.array(pxs), delimiter=" ", header=header)
np.savetxt("../data/output/" + foldername + "/" + filename_prefix + "_yp.csv", yps, delimiter=" ", header=header)
np.savetxt("../data/output/" + foldername + "/" + filename_prefix + "_e.csv", es, delimiter=" ", header=header)