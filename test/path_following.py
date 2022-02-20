from scipy.integrate import odeint
from scipy.spatial import KDTree
import numpy as np

from vehicle_dynamics.vehicle_params import VehicleParam
from vehicle_dynamics.vehicle_dynamics import vehicle_dynamics

road = np.loadtxt("laneChangePath.txt", delimiter=" ")
tree = KDTree(road[:, :2])

def sim():
  tStart = 0  # start time
  tFinal = 8  # start time

  us = []
  yes = []
  phies = []


  def calc_distance(pa, pb, pv): 
    u = np.array([pb[0] - pa[0], pb[1] - pa[1]])
    v = np.array([pv[0] - pa[0], pv[1] - pa[1]])
    L = np.cross(u, v) / np.linalg.norm(u)
    return L


  def func_vehicle_dynamics(x, t, p):
    _, id = tree.query((x[0], x[1]))
    kp = 1.0
    ky = 0.05
    phie = x[2] - road[id, 2]
    ye = calc_distance([road[id, 0], road[id, 1]], [road[min(id+1, len(road)-1), 0], road[min(id+1, len(road)-1), 1]], [x[0], x[1]])
    u = [-x[5]*x[4]*1600,  - kp*phie - ky*ye]
    us.append(u)
    yes.append(ye)
    phies.append(phie)
    f = vehicle_dynamics(x, u, p)
    return f


  # load vehicle parameters
  p = VehicleParam()

  # initial state for simulation
  delta0 = 0
  vel0 = 20
  Psi0 = 0
  sy0 = 0
  x0 = [0, 0, 0, 20, 0, 0, 0]

  t = np.arange(0, tFinal, 0.01)
  x = odeint(func_vehicle_dynamics, x0, t, args=(p,))
