import numpy as np

class AdaptiveController:
  
  def __init__(self, ref, w1, w2, gamma):
    self.ref = ref
    self.w1 = w1
    self.w2 = w2
    param_num = w1.sys.noutputs + w2.sys.noutputs + ref.sys.noutputs + 1
    self.theta = np.zeros((param_num, 1))
    self.theta[-1] = 1.0
    self.gamma = gamma*np.identity(param_num)

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
    return ""    

class AdaptiveControllerN2(AdaptiveController):

  def __init__(self, ref, w1, w2, phi, gamma):

    super(AdaptiveControllerN2, self).__init__(ref, w1, w2, gamma)
    self.phi = phi

  def update(self, dt, yp, r):

    u = self.calc_u(yp, r)
    self.ref.update(dt, r)
    self.w1.update(dt, np.repeat(u, self.w1.sys.ninputs, axis=0))
    self.w2.update(dt, np.repeat(yp, self.w2.sys.ninputs, axis=0))
    w = np.array([np.hstack([self.w1.state.T[0], self.w2.state.T[0], yp[0], r[0]])]).T
    self.phi.update(dt, w)

    e = yp - self.ref.observe()

    ## update params
    self.theta = self.theta - e*self.gamma.dot(self.phi.state)*dt

  def calc_u(self, yp, r, verbose=False):

    e = yp - self.ref.observe()
    w = np.array([np.hstack([self.w1.state.T[0], self.w2.state.T[0], yp[0], r[0]])]).T
    u = self.theta.T.dot(w) - self.phi.state.T.dot(self.gamma).dot(self.phi.state)*e

    return np.array(u)# np.clip(np.array(u), a_min=-0.1, a_max=0.1)

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
    return ""

