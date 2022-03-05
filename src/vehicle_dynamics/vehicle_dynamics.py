import numpy as np
from math import *


def vehicle_dynamics(state, input, param):
  '''
  [status]
    - 0: x position in a global coordinate system [m]
    - 1: y position in a global coordinate system [m]
    - 2: yaw angle [rad]
    - 3: longitudinal velocity [m/s]
    - 4: lateral velocity [m/s]
    - 5: yaw rate [rad/s]
    - 6: steering angle [rad]
  [input]
    - 0: longitudinal acceleration
    - 1: steering angle
  '''

  x = state[0]
  y = state[1]
  phi = state[2]
  vx = state[3]
  vy = state[4]
  dphi = state[5]
  delta = state[6]

  fx = input[0]
  dref = input[1]

  rhs = [
    vx*cos(phi) - vy*sin(phi),  # x
    vx*sin(phi) + vy*cos(phi), # y
    dphi, # phi
    dphi*vy + fx/param.mass, # vx
    -(param.cf + param.cr)/param.mass * vy/vx + dphi*((-param.lf*param.cf + param.lr*param.cr)/param.mass/vx -vx) + param.cf/param.mass*dref, # vy
    (-param.lf*param.cf+param.lr*param.cr)/param.Iz/vx*vy - (param.lf*param.lf*param.cf+param.lr*param.lr*param.cr)/param.Iz/vx*dphi + param.lf*param.cf/param.Iz*dref, # dphi
    dref,
  ]

  return rhs
