import numpy as np
import matplotlib.pyplot as plt
from control.matlab import *

from linear_system import *
from vehicle_params import *
from vehicle_dynamics import *
from mrac.parameter_estimator import *
from mrac.controller_designer import *

import warnings
warnings.simplefilter('ignore')


def bode_and_pz(system):
  zeros = zero(system)
  zeros_x = zeros.real
  zeros_y = zeros.imag

  poles = pole(system)
  poles_x = poles.real
  poles_y = poles.imag

  mag, phase, omega = bode(system, dB=False, Hz=True)

  return (zeros, poles), (mag, phase, omega)


def plant_bode_and_pz(plant_sys, param):

  dim = int((len(param)- 1) / 2.0)
  num, den = current_tf(plant_sys, param.T[0], dim, lbd)
  est_tf = tf(num, den)

  return bode_and_pz(est_tf)

def outfile(file, pz, bode):

  np.savetxt(file+"_zero.txt", np.vstack([pz[0].real, pz[0].imag]).T, delimiter=" ")
  np.savetxt(file+"_pole.txt", np.vstack([pz[1].real, pz[1].imag]).T, delimiter=" ")

  np.savetxt(file+"_bode.txt", np.vstack([bode[2], bode[0], bode[1]]).T, delimiter=" ")


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
  
elif plant_type == "small": ## nissan cube
  plantParam.mass = 1180.0
  plantParam.cf = 90000.0
  plantParam.cr = 90000.0
  plantParam.lf = 1.05
  plantParam.lr = 1.48
  plantParam.Iz = 1945.0

elif plant_type == "van":
  plantParam.mass = 3040.0
  plantParam.cf = 90000.0
  plantParam.cr = 90000.0
  plantParam.lf = 1.85
  plantParam.lr = 1.895
  plantParam.Iz = 10660.0

else:
  plant_type = "prius"

thetas =  []

## Reference model
modelParam = VehicleParam()
A, B, C, D = linear_vehicle_model_fb(modelParam, vx,  -2.0, -0.05)
mss = ss(A, B, C, D)

A, B, C, D = linear_vehicle_model(modelParam, vx)
tss = ss(A, B, C, D)

A, B, C, D = linear_vehicle_model(plantParam, vx)
pss = ss(A, B, C, D)

lbd0 = [1, 1]
sv_dim, L, l = designed_state_space_eq(mss, lbd0)
lbd = np.hstack([[1.0], -L[0]])
param = estimate_theta(tss, mss, sv_dim, lbd0)

estTheta = estimate_theta(tss, mss, sv_dim, lbd0)
initTheta = np.zeros_like(estTheta)
initTheta[-1] = 1.0

ref_data = bode_and_pz(mss)
outfile("ref", ref_data[0], ref_data[1])

zeroini_data = plant_bode_and_pz(pss, initTheta)
outfile("zero", zeroini_data[0], zeroini_data[1])

estini_data = plant_bode_and_pz(pss, estTheta)
outfile("est", estini_data[0], estini_data[1])

A, B, C, D = linear_vehicle_model_fb(plantParam, vx,  0.0, -0.05)
fbpss = ss(A, B, C, D)
fbplant_data = bode_and_pz(fbpss)
outfile("fb", fbplant_data[0], fbplant_data[1])