import numpy as np
from math import *
import copy

class AdaptiveController:
  
  def __init__(self, ref, w1, w2, gamma, umin = None, umax = None):
    self.ref = ref
    self.w1 = w1
    self.w2 = w2
    param_num = w1.sys.noutputs + w2.sys.noutputs + ref.sys.noutputs + 1
    self.theta = np.zeros((param_num, 1))
    self.theta[-1] = 1.0
    if type(gamma) is np.ndarray:
      self.gamma = np.diag(gamma)
    elif type(gamma) is int or type(gamma) is float:
      self.gamma = gamma*np.identity(param_num)


    self.umin = umin
    self.umax = umax

  def update(self, dt, yp, r):

    self.ref.update(dt, r)
    self.w1.update(dt, self.calc_u(yp, r))
    self.w2.update(dt, yp)
    e = yp - self.ref.observe()

    ## update params
    w = np.array([np.hstack([self.w1.state.T[0], self.w2.state.T[0], yp[0], r[0]])]).T
    self.theta = self.theta - e*self.gamma.dot(w)*dt

  def calc_u(self, yp, r, verbose=False):
    w = np.array([np.hstack([self.w1.state.T[0], self.w2.state.T[0], yp[0], r[0]])]).T
    u = self.theta.T.dot(w)

    if (self.umin is not None and self.umax is not None):
      return np.clip(np.array(u), self.umin, self.umax)

    return np.array(u)

  def safe_r(self, yp, r):
    return r

  def __repr__(self):
    return self.__str__()

  def __str__(self):
    print("Reference model:")
    print(self.ref)
    print("System Variables:")
    print(self.w1)
    print(self.w2)
    print("Adaptive parameter")
    print(" gamma: ")
    print(self.gamma)

    return ""    

class AdaptiveControllerN2(AdaptiveController):
  '''
    Iannou and sun Table 6.2
  '''

  def __init__(self, ref, w1, w2, phi, gamma, umin = None, umax = None):

    super(AdaptiveControllerN2, self).__init__(ref, w1, w2, gamma, umin, umax)
    self.phi = phi

  def update(self, dt, yp, r, u):

    w = np.array([np.hstack([self.w1.state.T[0], self.w2.state.T[0], yp[0], r[0]])]).T

    self.ref.update(dt, r)
    self.w1.update(dt, np.repeat(u, self.w1.sys.ninputs, axis=0))
    self.w2.update(dt, np.repeat(yp, self.w2.sys.ninputs, axis=0))
    self.phi.update(dt, w)

    e = yp - self.ref.observe()

    ## update params
    self.theta = self.theta - e*self.gamma.dot(self.phi.state)*dt

  def calc_u(self, yp, r, verbose=False):

    e = yp - self.ref.observe()
    w = np.array([np.hstack([self.w1.state.T[0], self.w2.state.T[0], yp[0], r[0]])]).T
    u = self.theta.T.dot(w) - self.phi.state.T.dot(self.gamma).dot(self.phi.state)*e

    if (self.umin is not None and self.umax is not None):
      return np.clip(np.array(u), self.umin, self.umax)

    return np.array(u)

  def safe_r(self, yp, r):
    e = yp - self.ref.observe()
    return -r*np.sign(e)

  def __repr__(self):
    return self.__str__()

  def __str__(self):
    print("Reference model:")
    print(self.ref)
    print("System Variables:")
    print(self.w1)
    print(self.w2)
    print("Adaptive parameter")
    print(" gamma: ")
    print(self.gamma)
    return ""

class ConstraintAdaptiveControllerN2(AdaptiveController):

  def __init__(self, ref, w1, w2, gamma, umin, umax, robust="nothing", rbParam = None):
    # robustness
    #  - "nothing"  : nothing
    #  - "deadzone" : dead zone, rbParam == limitation of disturbance
    if robust == "deadzone" and rbParam is None:
      rbParam = 1.0
    #  - "thetamax" : bounded theta, rbParam == limitaion of theta
    if robust == "thetamax" and rbParam is None:
      rbParam = 5000.0
    #  - "s-mod"    : sigma-modification, rbParam == sigma
    if robust == "s-mod" and rbParam is None:
      rbParam = 1.0
    #  - "e-mod"    : error modification, rbParam == factor
    if robust == "e-mod" and rbParam is None:
      rbParam = 1.0

    self.robust = robust
    self.rbParam = rbParam

    super(ConstraintAdaptiveControllerN2, self).__init__(ref, w1, w2, gamma, umin, umax)
    phi_dim = w1.sys.noutputs + w2.sys.noutputs + ref.sys.ninputs + ref.sys.noutputs

    self.phi = []
    for i in range(phi_dim):
      self.phi.append(copy.deepcopy(ref))
    
    self.theta_phi = copy.deepcopy(ref)
    self.k1 = 1.0
    self.xi = copy.deepcopy(ref)
    self.kd = np.array([0.0])
    self.du = np.array([[0.0]])
    self.e1 = 0.0
    self.e2 = 0.0

  def update(self, dt, yp, r, u):

    phi = []
    for p in self.phi:
      phi.append(p.observe()[0])
    phi = np.array(phi)

    w = np.array([np.hstack([self.w1.state.T[0], self.w2.state.T[0], yp[0], r[0]])]).T
    xi = self.xi.observe()

    e2 = self.theta.T.dot(phi) - self.theta_phi.observe()
    e = yp - self.ref.observe()
    e1 = e - self.kd.dot(xi) + e2*self.k1

    self.ref.update(dt, r)
    self.w1.update(dt, np.repeat(u, self.w1.sys.ninputs, axis=0))
    self.w2.update(dt, np.repeat(yp, self.w2.sys.ninputs, axis=0))
    self.theta_phi.update(dt, self.theta.T.dot(w))
    self.xi.update(dt, self.du)
    for idx, wb in enumerate(w):
      self.phi[idx].update(dt, np.array([wb]))

    ## update params
    dthetadt = - e1*self.gamma.dot(phi)/(1 + phi.T.dot(phi))
    dk1dt = - e1*e2/(1 + phi.T.dot(phi))
    dkddt = e1*xi/(1 + phi.T.dot(phi))
    if self.robust == "deadzone":
      if (abs(e1) < self.rbParam) :
        dthetadt = np.zeros_like(dthetadt)
        dkddt = np.zeros_like(dkddt)
        dk1dt = np.zeros_like(dk1dt)
    elif self.robust == "s-mod":
      dthetadt -= self.rbParam*self.theta/(1 + phi.T.dot(phi))
    elif self.robust == "e-mod":
      dthetadt -= self.rbParam*abs(e1)*self.theta/(1 + phi.T.dot(phi))
    else:
      pass

    self.theta = self.theta + dthetadt*dt
    self.k1 = self.k1 + dk1dt*dt
    self.kd = self.kd + dkddt*dt
    self.e2 = e2
    self.e1 = e1

  def calc_u(self, yp, r):

    w = np.array([np.hstack([self.w1.state.T[0], self.w2.state.T[0], yp[0], r[0]])]).T

    '''
    phi = []
    for p in self.phi:
      phi.append(p.observe()[0])
    phi = np.array(phi)

    e2 = self.theta.T.dot(phi) - self.theta_phi.observe()
    e = yp - self.ref.observe()
    e1 = e + e2*self.k1
    '''

    v = self.theta.T.dot(w) #- e1*phi.T.dot(phi)/(1 + phi.T.dot(phi))
    u = np.clip(np.array(v), self.umin, self.umax)
    self.du = np.array(u - v)

    return np.array(u)
