
from control.matlab import *
import matplotlib.pyplot as plt 
import numpy as np

from mrac.parameter_estimator import current_tf 
from vehicle_params import *
from vehicle_dynamics import *
from mrac.parameter_estimator import *
from mrac.controller_designer import designed_state_space_eq

vx = 10.0
lbd0 = [1, 1]

## Reference Model
modelParam = VehicleParam()
A, B, C, D = linear_vehicle_model_fb(modelParam, vx, -2.0, -0.05)
mss = ss(A, B, C, D)

print("")
print("[reference tf]")
print(tf(mss))

p, z = pzmap(mss)
print("poles: ")
print(p)
print("zeros: ")
print(z)
plt.savefig("ref.png")
plt.clf()


## plant model
plantParam = VehicleParam()
plantParam.mass = 1727.0
plantParam.cf = 94000.0
plantParam.cr = 94000.0
plantParam.lf = 1.17
plantParam.lr = 1.42
plantParam.Iz = 2867.0

A, B, C, D = linear_vehicle_model(plantParam, vx)
plant_sys = ss(A, B, C, D)

print("[plant tf]")
print(tf(plant_sys))
p, z = pzmap(tf(plant_sys))
plt.savefig("plant.png")
plt.clf()

## estimated 
n, L, _ = designed_state_space_eq(mss, lbd0)
lbd = np.hstack([[1.0], -L[0]])

'''
theta = [
  3.429623364021999743e-01,
  -1.139104905654302752e-02,
  -6.515285068214244149e-03,
  -2.559308896660652533e-01,
  5.903124272104456199e-03,
  5.368753834682423419e-03,
  -2.063821925026322113e+00,
  1.000000000000000000e+00,
]
'''

# est
estTheta = estimate_theta(plant_sys, mss, n, lbd0)

theta = [ 1.47456755e+01,
  6.70046300e+02,
  2.13642128e+03,
 -6.28186160e+00,
  2.81400058e+03,
  2.62475394e+03,
 -1.41515746e+01,
  1.11347518e+00
]

print("")
print("[current param]")
print(theta)

num, den = current_tf(plant_sys, theta, n, lbd)
est_tf = tf(num, den)

print("")
print("[estimated tf]")
print(est_tf)

plt.clf()
p, z = pzmap(est_tf)
print("poles: ")
print(p)
print("zeros: ")
print(z)
plt.savefig("est.png")
