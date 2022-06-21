import numpy as np
from scipy.spatial import KDTree
from scipy.integrate import odeint

from vehicle_dynamics import linear_vehicle_model
from util import *
from bezier import *

class VehicleLTI: 

  def __init__(self, vehicleParam, init, vx, road):
    self.vehicle_param = vehicleParam
    self.x = np.array([init]).T 
    self.road = road
    self.tree = KDTree(road[:, :2])
    self.vx = vx

  def dynamics(self, state, input):
    param = self.vehicle_param
    vx = self.vx

    A, B, _, _ = linear_vehicle_model(param, vx)

    dsdt = np.array(A).dot(state) + np.array(B).T[0] * input[0][0]
    return dsdt

  def update(self, dt, input):

    def func_vehicle_dynamics(x, t, input):
      u = input
      f = self.dynamics(x, u)
      return f

    t = np.arange(0, dt, dt/10)
    x = odeint(func_vehicle_dynamics, self.x, t, args=(input,))
    self.x = x[-1]

  def observe(self):
    param = self.vehicle_param
    vx = self.vx    
    _, _, C, _ = linear_vehicle_model(param, vx)

    return np.array([np.array(C).dot(self.x)])