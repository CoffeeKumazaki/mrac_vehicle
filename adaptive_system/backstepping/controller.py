import numpy as np


class BacksteppingController:
  """
  Adaptive Backstepping Controller for relative degree 2 systems.
  Uses K-filter framework (same as MRAC) with 2-step backstepping
  design and tuning functions for parameter adaptation.

  The control law uses the same regressor structure as MRAC:
    u = theta^T * omega
  but the adaptation law is derived from a backstepping Lyapunov design
  incorporating both z1 (tracking error) and z2 (filtered derivative error).

  This avoids the division-by-estimated-parameter problem while maintaining
  the backstepping stability guarantee.
  """

  def __init__(self, ref, xi, vi, gamma, pgamma, c1=1.0, c2=2.0, umin=None, umax=None):
    self.ref = ref
    self.xi = xi    # xi filter (yp side)
    self.vi = vi    # v filter (u side)

    self.c1 = c1
    self.c2 = c2
    self.gamma = gamma
    self.pgamma = pgamma
    self.umin = umin
    self.umax = umax

    sv_dim = xi.sys.noutputs
    self.sv_dim = sv_dim

    # Parameter vector: [theta_1 (xi filter); theta_2 (vi filter); theta_yp; theta_r]
    # Same structure as MRAC: theta^T * omega where omega = [xi.state; vi.state; yp; r]
    param_num = sv_dim * 2 + ref.sys.noutputs + ref.sys.ninputs
    self.theta = np.zeros((param_num, 1))
    self.theta[-1] = 1.0  # initial gain for r

    if type(gamma) is np.ndarray:
      self.gamma_mat = np.diag(gamma)
    elif type(gamma) is int or type(gamma) is float:
      self.gamma_mat = gamma * np.identity(param_num)

    # Extract L, l from filter state-space
    self.L = np.array(xi.sys.A)
    self.l = np.array(xi.sys.B)

    # Store previous yr for numerical derivative
    self._yr_prev = 0.0
    self._z1_prev = 0.0

  def _get_omega(self, yp, r):
    """Build regressor vector omega = [xi.state; vi.state; yp; r]."""
    return np.array([np.hstack([
      self.xi.state.T[0],
      self.vi.state.T[0],
      yp[0],
      r[0]
    ])]).T

  def step1(self, yp, yr):
    """
    Step 1: Compute tracking error z1 and stabilizing function alpha1.
    z1 = yp - yr
    """
    z1 = yp[0, 0] - yr
    return z1

  def step2(self, z1, z1_dot_est):
    """
    Step 2: Compute z2 from filtered error derivative.
    z2 = z1_dot_est + c1*z1 (deviation from desired z1_dot = -c1*z1)
    The desired dynamics is z1_dot = -c1*z1, so z2 measures how far
    we are from achieving that.
    """
    z2 = z1_dot_est + self.c1 * z1
    return z2

  def update(self, dt, yp, r, u=None):
    """
    Update filters, reference model, and adaptive parameters.
    Must be called AFTER calc_u in the simulation loop.
    """
    # Update reference model
    self.ref.update(dt, r)

    # Update filters
    u_actual = u if u is not None else np.array([[0.0]])
    self.xi.update(dt, np.repeat(u_actual, self.xi.sys.ninputs, axis=0))
    self.vi.update(dt, np.repeat(yp, self.vi.sys.ninputs, axis=0))

    # Get current reference output
    yr_obs = self.ref.observe()
    yr = yr_obs.flat[0]

    # Step 1: tracking error
    z1 = self.step1(yp, yr)

    # Filtered derivative of z1 (numerical differentiation)
    z1_dot_est = (z1 - self._z1_prev) / dt
    self._z1_prev = z1

    # Step 2: derivative error
    z2 = self.step2(z1, z1_dot_est)

    # Regressor
    omega = self._get_omega(yp, r)

    # Backstepping tuning function adaptation law:
    # theta_dot = -Gamma * (c1*z1 + c2*z2) * omega
    # This comes from the Lyapunov function V = 0.5*z1^2 + 0.5*z2^2 + 0.5*theta_tilde^T*Gamma^-1*theta_tilde
    error_signal = self.c1 * z1 + self.c2 * z2
    dtheta = -self.gamma_mat.dot(omega) * error_signal * dt
    self.theta = self.theta + dtheta

    self._yr_prev = yr

  def calc_u(self, yp, r):
    """
    Compute control input using adaptive backstepping.
    u = theta^T * omega - c1*z1 - c2*z2
    The additional -c1*z1 - c2*z2 terms provide backstepping stabilization
    beyond what the adaptive parameters alone achieve.
    """
    # Get reference output
    yr_obs = self.ref.observe()
    yr = yr_obs.flat[0]

    # Step 1: tracking error
    z1 = self.step1(yp, yr)

    # Filtered derivative of z1
    z1_dot_est = (z1 - self._z1_prev) / 0.01  # dt approximation

    # Step 2: derivative error
    z2 = self.step2(z1, z1_dot_est)

    # Regressor
    omega = self._get_omega(yp, r)

    # Control law: adaptive term + backstepping stabilization
    u_adaptive = self.theta.T.dot(omega)
    u_bs = -self.c1 * z1 - self.c2 * z2
    u = u_adaptive + u_bs

    # Apply saturation
    if self.umin is not None and self.umax is not None:
      u = np.clip(u, self.umin, self.umax)

    return np.array(u)
