from scipy.spatial import KDTree
from scipy.integrate import odeint
import random 

from vehicle_dynamics import vehicle_dynamics
from util import *
from bezier import *

class Vehicle: 

  def __init__(self, vehicleParam, init, road, noise_dev = 0.0):
    self.vehicle_param = vehicleParam
    self.x = init # [0, 0, 0, 20, 0, 0, 0]
    self.road = road
    self.tree = KDTree(road[:, :2])
    self.noise = noise_dev

  def update(self, dt, input):

    def func_vehicle_dynamics(x, t, p, input):
      u = input
      f = vehicle_dynamics(x, u, p)
      return f

    t = np.arange(0, dt, dt/10)
    x = odeint(func_vehicle_dynamics, self.x, t, args=(self.vehicle_param, input))
    self.x = x[-1]

  def observe(self, noise = True):

    def isLeft(closest, vehicle, target):
      u = np.array([target[0] - closest[0], target[1] - closest[1]])
      v = np.array([vehicle[0] - closest[0], vehicle[1] - closest[1]])

      return np.cross(u, v) > 0

    cur_pos = (self.x[0], self.x[1])
    _, id = self.tree.query(cur_pos)
    id = np.clip(id, 1, len(self.road-2))
    bezier_points = self.road[id-1:id+3, 0:2]
    bezier = Bezier(bezier_points)
    closest, ang, d = bezier.closest(cur_pos)
    kp = 1.0
    ky = 0.05
    phie = self.x[2] - ang
    ye = d
    if not isLeft(closest, cur_pos, bezier_points[-1]):
      ye *= -1

    if noise:
      ye += random.gauss(0.0, self.noise)
    return np.array([[ye]])