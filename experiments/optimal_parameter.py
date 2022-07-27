
import numpy as np
import matplotlib.pyplot as plt
from control.matlab import *

from linear_system import *
from vehicle_params import *
from vehicle_dynamics import *
from mrac.parameter_estimator import *
from mrac.controller_designer import *


def plot_step(sys, file = None):
  plt.cla()
  file = file if file is not None else "step.png"
  (y1a, T1a) = step(sys,T = np.arange(0, 2, 0.01))
  plt.plot(T1a, y1a[:, 0])
  plt.savefig(file)

plantParam = VehicleParam()

vx = 20
plant_type = "vehicle"

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

## Reference model
modelParam = VehicleParam()
A, B, C, D = linear_vehicle_model_fb(modelParam, vx,  -2.0, -0.05)
mss = ss(A, B, C, D)
ref = LinearSystem(mss, 0, "ref")

thetas = []

for i in np.arange(0.5, 3.0, 0.1):
  lbd0 = [1, 1]
  sv_dim, L, l = designed_state_space_eq(mss, lbd0)

  # plot_step(ss(L, l, np.identity(sv_dim), 0), f"../data/output/step_{i}.png")

  ## plant LTI model
  plantParam.cf = 94000.0*i
  plantParam.cr = 94000.0*i
  A, B, C, D = linear_vehicle_model(plantParam, vx)
  pss = ss(A, B, C, D)

  estTheta = estimate_theta(pss, mss, sv_dim, lbd0)
  thetas.append(np.hstack([[i], estTheta.T[0]]))


np.savetxt(f"../data/output/{plant_type}_cfcr_params.csv", np.array(thetas), delimiter=" ")