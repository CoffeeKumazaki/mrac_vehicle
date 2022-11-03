import matplotlib.pyplot as plt 
from sympy import *

from control import *
from control.matlab import *
from vehicle_params import *
from vehicle_dynamics import *
from mrac.parameter_estimator import *
from mrac.controller_designer import *

vx = 15

## Reference model
modelParam = VehicleParam()
A, B, C, D = linear_vehicle_model(modelParam, vx)
mss = ss(A, B, C, D)

lbd0 = [1, 1]
sv_dim, L, l = designed_state_space_eq(mss, lbd0)

tf_lambda = tf(ss(L, l, np.eye(sv_dim), 0))
nlam, dlam = tfdata(tf_lambda)

## plant LTI model
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


A, B, C, D = linear_vehicle_model(plantParam, vx)
pss = ss(A, B, C, D)
pnum, pden = tfdata(pss)

trueTheta = estimate_theta(pss, mss, sv_dim, lbd0)
initTheta = np.zeros_like(trueTheta)
initTheta[-1] = 1.0
tildeTheta = initTheta - trueTheta

s = symbols('s')
Dp = Poly(pden[0][0], s)
Np = Poly(pnum[0][0], s)
Dlambda = Poly(dlam[0][0], s)

# 分母
theta1 = Poly(trueTheta[0:3].T[0], s)
theta2 = Poly(trueTheta[3:6].T[0], s)
theta3 = Poly(trueTheta[6:7].T[0], s)

F1 = Dlambda - theta1
F2 = theta2 + Dlambda*theta3
Dm = (Dlambda * (F1 * Dp - Np * F2) )

# 分子
theta1 = Poly(tildeTheta[0:3].T[0], s)
theta2 = Poly(tildeTheta[3:6].T[0], s)
theta3 = Poly(tildeTheta[6:7].T[0], s)

# print(Dlambda)
# print(theta1)
# print(theta2)
# print(theta3)

F1 = -theta1
F2 = theta2 + Dlambda*theta3

DeltaD = (Dlambda * (F1 * Dp - Np * F2) )

nn = []
dn = []
for d in DeltaD.all_coeffs():
  nn.append(d)

for d in Dm.all_coeffs():
  dn.append(d)

# print(nn)
# print(dn)
small_gain = tf(np.array(nn).astype(float), np.array(dn).astype(float))

print(DeltaD - Dm)

plt.clf()
print(small_gain)
mag, phase, omega = bode(small_gain, plot=False, dB=False, Hz=True)

data = np.stack([omega, mag])
np.savetxt("../data/output/sg_truck_vs"+str(vx)+".dat", data.T, delimiter=" ")

plt.xscale('log')
plt.yscale('log')

plt.xlim(0.001, 100)
plt.ylim(0.01, 2)
plt.grid(which='both', axis='y')
plt.grid(which='major', axis='x')
plt.plot(omega, mag)
plt.savefig("sg_truck_vs"+str(vx)+".png")