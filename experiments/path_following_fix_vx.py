import tqdm
import numpy as np
from control.matlab import *
from scipy.spatial import KDTree

from linear_system import *
from vehicle_params import *
from vehicle_dynamics import *
from vehicle import *
from mrac.controller import *
from mrac.parameter_estimator import *

def sim(simT, dt, plant, controller):
  
  Ts = np.arange(0,simT, dt)

  for t in tqdm.tqdm(Ts):

    r = np.array([[0.0]])

    yp = plant.observe()

    u = adaptive_ctrl.calc_u(yp, r)
    u = np.clip(u, -1.0, 1.0)
    xp = plant.x
    plant.update(dt, [-xp[5]*xp[4]*plant.vehicle_param.mass, u])
    #plant.update(dt, u)

    controller.update(dt, yp, r)

    yr = controller.ref.observe()

    e = yp[0]-yr[0]
    theta = controller.theta.T[0]
    yield t, yp[0], yr[0], e, theta, u[0], r[0], plant.x


road = np.loadtxt("../data/path.txt", delimiter=" ", dtype=np.float32)
tree = KDTree(road[:, :2])

plantParam = VehicleParam()
'''
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

vx = 5.0
plantInit = [road[0,0], road[0,1], road[0,2], vx, 0, 0, 0]
## Plant model
#A, B, C, D = vehicleModel(plantParam, vx)
A, B, C, D = linear_vehicle_model_fb(plantParam, vx, -1.0, -0.05)
pss = ss(A, B, C, D)
plant_lti = LinearSystem(pss, 0, "plant")


## Reference model
modelParam = VehicleParam()
A, B, C, D = linear_vehicle_model_fb(modelParam, vx, -2.0, -0.05)
mss = ss(A, B, C, D)
ref = LinearSystem(mss, 0, "ref")

sv_dim = 3
L = np.zeros((sv_dim, sv_dim))
l = np.zeros((sv_dim, 1))
L[0, 0] = -15.8965517241379
L[0, 1] = -201.103448275861
L[0, 2] = -186.206896551724
L[1, 0] = 1.0 
L[2, 1] = 1.0
l[0, 0] = 1.0
w1 = LinearSystem(ss(L, l, np.identity(sv_dim), 0), 0, "w1")
w2 = LinearSystem(ss(L, l, np.identity(sv_dim), 0), 0, "w2")


phi_dim = sv_dim*2 + ref.sys.ninputs + ref.sys.noutputs
Lp = -np.identity(phi_dim)*1
lp = np.identity(phi_dim)*1
phi = LinearSystem(ss(Lp, lp, np.identity(phi_dim), 0), 0, 'phi')

adaptive_ctrl = AdaptiveControllerN2(ref, w1, w2, phi, 10.0)

# set estimated param

## plant LTI model
A, B, C, D = linear_vehicle_model(plantParam, vx)
pss = ss(A, B, C, D)

print(tf(pss))
print(tf(mss))

estTheta = estimate_theta(pss, mss, 3, [1,1])
print(estTheta)
adaptive_ctrl.theta[-2] = -0.05
# adaptive_ctrl.theta = estTheta

plant = Vehicle(plantParam, plantInit, road)

res = sim(180, 0.01, plant, adaptive_ctrl)


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
  
  except: 
    print("finish")
    break


np.savetxt("../data/output/no_init_param/adaptive_track_vx_05_gain_10_theta.csv", np.array(thetas), delimiter=" ")
np.savetxt("../data/output/no_init_param/adaptive_track_vx_05_gain_10_u.csv", np.array(us), delimiter=" ")
np.savetxt("../data/output/no_init_param/adaptive_track_vx_05_gain_10.csv", np.array(pxs), delimiter=" ")
np.savetxt("../data/output/no_init_param/adaptive_track_vx_05_gain_10_yp.csv", yps, delimiter=" ")
np.savetxt("../data/output/no_init_param/adaptive_track_vx_05_gain_10_e.csv", es, delimiter=" ")
#np.savetxt("../data/output/no_init_param/adaptive_track.csv", np.array(res["plant"]), delimiter=" ")
#np.savetxt("../data/output/no_init_param/adaptive_track_yp.csv", np.array(res["yp"]), delimiter=" ")
#np.savetxt("../data/output/no_init_param/adaptive_track_theta.csv", np.array(res["theta"]), delimiter=" ")