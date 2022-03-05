from scipy.spatial import KDTree
from scipy.integrate import odeint

from vehicle_dynamics import vehicle_dynamics
from util import *

class Vehicle: 

  def __init__(self, vehicleParam, init, road):
    self.vehicle_param = vehicleParam
    self.x = init # [0, 0, 0, 20, 0, 0, 0]
    self.road = road
    self.tree = KDTree(road[:, :2])

  def update(self, dt, input):

    def func_vehicle_dynamics(x, t, p, input):
      u = input
      f = vehicle_dynamics(x, u, p)
      return f

    t = np.arange(0, dt, dt/10)
    x = odeint(func_vehicle_dynamics, self.x, t, args=(self.vehicle_param, input))
    self.x = x[-1]

  def observe(self):

    road = self.road
    _, id = self.tree.query((self.x[0], self.x[1]))
    id = max(0, min(999, id))
    phie = self.x[2] - road[id, 2]
    ye = calc_distance([road[id, 0], road[id, 1]], [road[min(id+1, len(road)-1), 0], road[min(id+1, len(road)-1), 1]], [self.x[0], self.x[1]])

    return np.array([[ye]])