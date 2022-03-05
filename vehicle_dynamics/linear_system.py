import numpy as np
from control.matlab import *

class LinearSystem:

  def __init__(self, sys, init, name="plant"):

    if isinstance(sys, TransferFunction):
      sys = tf2ss(sys)

    self.sys = sys

    if type(init) is int:
      self.state = np.ones((sys.nstates, 1)) * init
    elif type(init) is list:
      if len(init) != sys.nstates:
        err_msg = f"init is wrong length {len(init)}: {sys.nstates} is expected"
        raise ValueError(err_msg)

      self.state = np.array([init]).T 
    elif type(init) is np.ndarray:
      if init.shape != (sys.nstates, 1):
        err_msg = f"init is wrong shape {init.shape}: ({sys.nstates}, 1) is expected"
        raise ValueError(err_msg)

      self.state = init
    else:
      err_msg = f"init is wrong shape {init.shape}: ({sys.nstates}, 1) is expected"
      raise ValueError(err_msg)

    self.name = name
    self.y = np.zeros((sys.noutputs, 1))
  
  def update(self, dt, input):

    resolution = 10
    T = np.arange(0, dt, dt/resolution)
    U = np.repeat(input, resolution, axis=1).T
    [y, t, x] = lsim(self.sys, T=T, X0=self.state, U=U)
    self.state = np.array([x[-1]]).T

    yp = np.array([[y[-1]]]) if type(y[-1]) is np.float64 else np.array([y[-1]])
    self.y = yp

  def observe(self):
    return self.y

  def __repr__(self):
    return self.__str__()

  def __str__(self):
    print("Transfer function:")
    print(ss2tf(self.sys))
    print("State space:")
    print(self.sys)
    print("Current state: ")
    print(self.state)
    return ""
