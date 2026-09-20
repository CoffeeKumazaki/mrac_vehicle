"""
Two-phase scenario for the electronic-towing study:
  Phase 1 (0-120 s)  : adaptation phase, in-lane zigzag (multi-sine, PE-rich)
  Phase 2 (120-300 s): highway-like driving
    120-140 straight / 140-175 gentle curve (enter-hold-exit)
    180-210 S-curve / 215-250 lane change and return / 250-280 sweeping curve

Variants:
  bs_adapt  : adaptive backstepping, zigzag adaptation phase, adaptation on
  bs_frozen : same, but adaptation frozen at t=120 s (tests "learned enough?")
  bs_cold   : no adaptation phase (r=0 until 120 s), adaptation on
  mrac      : AdaptiveControllerN2 with the zigzag adaptation phase
"""
from math import sin, pi
import copy
import tqdm
import numpy as np
from control.matlab import *

from linear_system import *
from vehicle_params import *
from vehicle_dynamics import *
from vehicle_LTI import *
from backstepping.controller import AdaptiveBacksteppingController
from mrac.controller import AdaptiveControllerN2
from mrac.controller_designer import designed_state_space_eq


def smoothstep(x):
  x = min(max(x, 0.0), 1.0)
  return x * x * (3.0 - 2.0 * x)


def reference_input(t, with_adaptation_phase=True):
  if t < 120.0:  # adaptation phase
    if not with_adaptation_phase:
      return np.array([[0.0]])
    r = 0.02 * sin(0.1 * t) + 0.015 * sin(0.35 * t) + 0.01 * sin(0.8 * t)
    return np.array([[float(r)]])
  # highway phase
  r = 0.0
  if 140.0 <= t < 175.0:    # gentle curve: enter 140-150, hold, exit 165-175
    r = 0.04 * (smoothstep((t - 140.0) / 10.0) - smoothstep((t - 165.0) / 10.0))
  elif 180.0 <= t < 210.0:  # S-curve
    r = 0.04 * sin(2.0 * pi * (t - 180.0) / 30.0)
  elif 215.0 <= t < 250.0:  # lane change (215-220) and return (245-250)
    r = 0.05 * (smoothstep((t - 215.0) / 5.0) - smoothstep((t - 245.0) / 5.0))
  elif 250.0 <= t < 280.0:  # sweeping curve
    r = 0.03 * sin(2.0 * pi * (t - 250.0) / 60.0)
  return np.array([[float(r)]])


SEGMENTS = [
  ("adaptation", 0.0, 120.0),
  ("straight", 120.0, 140.0),
  ("curve", 140.0, 175.0),
  ("s_curve", 180.0, 210.0),
  ("lane_change", 215.0, 250.0),
  ("sweep", 250.0, 280.0),
]


def sim(simT, dt, plant, controller, with_adaptation_phase, freeze_at=None):
  frozen = False
  for t in tqdm.tqdm(np.arange(0, simT, dt)):
    if freeze_at is not None and not frozen and t >= freeze_at:
      controller.gamma_mat = np.zeros_like(controller.gamma_mat)
      controller.gamma_rho = 0.0
      controller.sigma = 0.0
      frozen = True

    r = reference_input(t, with_adaptation_phase)
    yp = plant.observe()
    u = controller.calc_u(yp, r)
    plant.update(dt, u)
    controller.update(dt, yp, r, u)

    yr = controller.ref.observe()
    yp = plant.observe()
    theta = controller.theta.T[0]
    yield t, yp[0], yr[0], yp[0] - yr[0], theta, u[0], r[0]


## Parameter settings
vx = 10
plant_type = "vehicle"
adaptive_gain = 1.0
umax = 100000.0
umin = -umax
simT = 300
dt = 0.01
lbd0 = [1, 1]  # MRAC filter design

filename_prefix = plant_type + "_lti_vx_10_bs_highway"

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

modelParam = VehicleParam()
A, B, C, D = linear_vehicle_model_fb(modelParam, vx, -2.0, -0.05)
mss = ss(A, B, C, D)

A, B, C, D = linear_vehicle_model(plantParam, vx)
pss = ss(A, B, C, D)


def make_bs(ref):
  return AdaptiveBacksteppingController(
    ref, n=4, gamma=adaptive_gain, sgn_bm=1.0,
    rho_hat0=modelParam.mass / modelParam.cf, umin=umin, umax=umax)


def make_mrac(ref):
  sv_dim, L, l = designed_state_space_eq(mss, lbd0)
  w1 = LinearSystem(ss(L, l, np.identity(sv_dim), 0), 0, "w1")
  w2 = LinearSystem(ss(L, l, np.identity(sv_dim), 0), 0, "w2")
  phi_dim = sv_dim * 2 + ref.sys.ninputs + ref.sys.noutputs
  phi = LinearSystem(ss(-np.identity(phi_dim), np.identity(phi_dim),
                        np.identity(phi_dim), 0), 0, 'phi')
  ctrl = AdaptiveControllerN2(ref, w1, w2, phi, adaptive_gain, umin=umin, umax=umax)
  ctrl.theta[-2] = -0.05
  ctrl.theta[-1] = 1.0
  return ctrl


VARIANTS = [
  # name, make_controller, with_adaptation_phase, freeze_at
  ("bs_adapt", make_bs, True, None),
  ("bs_frozen", make_bs, True, 120.0),
  ("bs_cold", make_bs, False, None),
  ("mrac", make_mrac, True, None),
]

foldername = "no_init_param"
header = f'''
  Two-phase towing scenario (see script docstring)
  Plant: {plant_type} / {plantParam}
  gain : {adaptive_gain}  vx : {vx}  umax : {umax}
  Reference: {modelParam}
'''

for name, make_ctrl, with_phase, freeze_at in VARIANTS:
  ref = LinearSystem(copy.deepcopy(mss), 0, "ref")
  plant = LinearSystem(copy.deepcopy(pss), [0, 0, 0, 0])
  ctrl = make_ctrl(ref)

  print(f"## variant: {name}")
  ts, yps, yrs, es, thetas, us, rs = [], [], [], [], [], [], []
  for t, yp, yr, e, theta, u, r in sim(simT, dt, plant, ctrl, with_phase, freeze_at):
    ts.append(t)
    yps.append(yp)
    yrs.append(yr)
    es.append(e)
    thetas.append(theta)
    us.append(u)
    rs.append(r)

  es_arr = np.array(es).flatten()
  for seg, lo, hi in SEGMENTS:
    s = slice(int(lo / dt), int(hi / dt))
    print(f"  {seg:12s} RMS e={np.sqrt(np.mean(es_arr[s]**2)):.4g}  max|e|={np.max(np.abs(es_arr[s])):.4g}")

  prefix = f"../data/output/{foldername}/{filename_prefix}_{name}"
  np.savetxt(prefix + "_yp.csv", yps, delimiter=" ", header=header)
  np.savetxt(prefix + "_yr.csv", yrs, delimiter=" ", header=header)
  np.savetxt(prefix + "_e.csv", es, delimiter=" ", header=header)
  np.savetxt(prefix + "_u.csv", us, delimiter=" ", header=header)
  np.savetxt(prefix + "_r.csv", rs, delimiter=" ", header=header)
  np.savetxt(prefix + "_theta.csv", np.array(thetas), delimiter=" ", header=header)

print("## done")
