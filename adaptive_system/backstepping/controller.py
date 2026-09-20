import numpy as np
from scipy.linalg import expm


class AdaptiveBacksteppingController:
  """
  Output-feedback adaptive backstepping with K-filters and tuning functions
  (Krstic, Kanellakopoulos, Kokotovic, "Nonlinear and Adaptive Control
  Design", 1995) for SISO LTI plants of known order n and relative degree 2.

  Plant assumptions:
    - known order n and relative degree rho = 2 (m = n - 2 zeros)
    - minimum phase (stable zeros)
    - known sign of the high-frequency gain b_m
    - all coefficients unknown: theta = [b_m .. b_0, a_{n-1} .. a_0]

  Plant in observer canonical form:
    x_dot = A0 x - a y + [0_{rho-1}; b] u,   y = x_1

  K-filters, with Abar = A0 - k e1^T Hurwitz (k from filter_poles):
    xi_dot  = Abar xi  + k y
    eta_dot = Abar eta + e_n y
    lam_dot = Abar lam + e_n u
    v_j = Abar^j lam (j = 0..m),   Xi column j = -Abar^{n-j} eta
  The state estimate x_hat = xi + Omega^T theta has observation error
  eps_dot = Abar eps, so no numerical differentiation is needed anywhere.

  Regressor (aligned with theta):
    omega = [v_{m,2}, .., v_{0,2}, row2(Xi) - y e1]
    y_dot = omega^T theta + xi_2 + eps_2

  Design (rho = 2, tuning functions):
    z1 = y - yr
    alpha1_bar = -(c1 + d1) z1 - xi_2 - omega_bar^T theta_bar_hat + yr_dot
    alpha1 = rho_hat alpha1_bar          (rho_hat estimates 1/b_m)
    z2 = v_{m,2} - alpha1
    rho_hat_dot = -gamma_rho sgn(b_m) alpha1_bar z1 / n_s
    tau2 = (omega - alpha1 e1) z1 - (dalpha1/dy) omega z2
    theta_hat_dot = Gamma tau2 / n_s
    n_s = 1 + nu (omega^T omega + z1^2 + z2^2)   (normalization)
    u = -b_m_hat z1 - (c2 + d2 (dalpha1/dy)^2) z2 - v_{m,3} + k_2 v_{m,1}
        + (dalpha1/dy)(xi_2 + omega^T theta_hat) + beta
  where beta collects the computable part of alpha1_dot (filter dynamics,
  reference derivatives and the update laws). With nu = 0 this is the pure
  tuning-function scheme:
    V = 1/2 z1^2 + 1/2 z2^2 + 1/2 theta_tilde^T Gamma^-1 theta_tilde
        + |b_m|/(2 gamma_rho) rho_tilde^2 + (eps terms)
    V_dot <= -c1 z1^2 - c2 z2^2
  so z1 -> 0 without persistent excitation and without dividing by any
  estimated parameter.

  The normalization n_s (nu > 0, standard robust-adaptive practice) bounds
  the effective adaptation rate so the Euler-discretized update stays stable
  for large signal amplitudes; beta always uses the actually applied update
  laws, so the control law remains consistent with the adaptation. Note the
  default configuration (nu > 0, sigma > 0) therefore trades the pure
  asymptotic-tracking proof above for discrete-time robustness; set
  nu = sigma = 0 to recover the textbook scheme (with a correspondingly
  small dt).

  Robust modifications (both standard; needed because the discrete-time
  implementation leaves a small non-vanishing tracking residual that acts as
  a bounded disturbance and otherwise causes parameter drift and bursting):
    - projection keeps rho_hat inside rho_bounds (sign-preserving, as in the
      original scheme where rho_hat is projected away from zero)
    - sigma-modification leaks theta_hat toward zero and rho_hat toward
      rho_hat0 at rate sigma

  The reference trajectory derivatives yr_dot, yr_ddot are computed exactly
  from the reference model state (Cm Am x_r etc.), assuming the reference
  model also has relative degree >= 2 (Cm Bm = 0).
  """

  def __init__(self, ref, n=4, filter_poles=-5.0, gamma=1.0, gamma_rho=None,
               c1=2.0, c2=2.0, d1=0.05, d2=0.05, nu=1.0, adapt_substeps=10,
               sigma=0.001, rho_bounds=None, sgn_bm=1.0,
               rho_hat0=0.02, theta_hat0=None, umin=None, umax=None):
    self.ref = ref
    self.n = n
    self.m = n - 2
    self.c1 = c1
    self.c2 = c2
    self.d1 = d1
    self.d2 = d2
    self.nu = float(nu)
    self.adapt_substeps = int(adapt_substeps)
    self.sgn_bm = float(np.sign(sgn_bm))
    self.umin = umin
    self.umax = umax

    # Reference model matrices for exact yr derivatives
    self.Am = np.asarray(ref.sys.A, dtype=float)
    self.Bm = np.asarray(ref.sys.B, dtype=float).reshape(-1)
    self.Cm = np.asarray(ref.sys.C, dtype=float).reshape(-1)
    if abs(self.Cm.dot(self.Bm)) > 1e-12:
      raise ValueError("reference model must have relative degree >= 2 (Cm Bm = 0)")

    # Observer gain k: char poly of Abar = A0 - k e1^T is s^n + k1 s^{n-1} + .. + kn
    if np.isscalar(filter_poles):
      poles = [filter_poles] * n
    else:
      poles = list(filter_poles)
      if len(poles) != n:
        raise ValueError(f"filter_poles needs {n} poles, got {len(poles)}")
    self.k = np.poly(poles)[1:].astype(float)

    A0 = np.eye(n, k=1)
    self.Abar = A0.copy()
    self.Abar[:, 0] -= self.k
    self.Abar_pows = [np.linalg.matrix_power(self.Abar, j) for j in range(n + 1)]

    # Filter states
    self.xi = np.zeros(n)
    self.eta = np.zeros(n)
    self.lam = np.zeros(n)

    # Adaptive states
    param_num = n + self.m + 1
    self.param_num = param_num
    if theta_hat0 is None:
      self.theta_hat = np.zeros(param_num)
    else:
      self.theta_hat = np.asarray(theta_hat0, dtype=float).reshape(-1).copy()
      if self.theta_hat.size != param_num:
        raise ValueError(f"theta_hat0 needs {param_num} entries")
    self.rho_hat = float(rho_hat0)
    self.rho_hat0 = float(rho_hat0)
    if self.rho_hat0 * self.sgn_bm <= 0.0:
      raise ValueError("rho_hat0 must have the sign of b_m")
    self.sigma = float(sigma)
    if rho_bounds is None:
      lo, hi = self.rho_hat0 / 20.0, self.rho_hat0 * 20.0
      self.rho_bounds = (min(lo, hi), max(lo, hi))
    else:
      self.rho_bounds = (float(rho_bounds[0]), float(rho_bounds[1]))

    if type(gamma) is np.ndarray:
      self.gamma_mat = np.diag(gamma)
      default_gamma_rho = float(np.mean(gamma))
    else:
      self.gamma_mat = float(gamma) * np.identity(param_num)
      default_gamma_rho = float(gamma)
    self.gamma_rho = default_gamma_rho if gamma_rho is None else float(gamma_rho)

    self._disc_cache = {}

  @property
  def theta(self):
    """[theta_hat; rho_hat] as a column vector, for logging like MRAC's theta."""
    return np.array([np.hstack([self.theta_hat, self.rho_hat])]).T

  def _discretization(self, dt):
    """ZOH discretization of the three K-filters, cached per dt."""
    if dt not in self._disc_cache:
      n = self.n
      Ad = expm(self.Abar * dt)
      BdI = np.linalg.solve(self.Abar, Ad - np.identity(n))
      en = np.zeros(n)
      en[-1] = 1.0
      self._disc_cache[dt] = (Ad, BdI.dot(self.k), BdI.dot(en))
    return self._disc_cache[dt]

  def _signals(self, yp, r):
    """All controller signals at the current instant (no state mutation)."""
    n, m = self.n, self.m
    y = float(np.asarray(yp).flat[0])
    rv = float(np.asarray(r).flat[0])
    k2 = self.k[1]

    # Reference trajectory and its exact derivatives from the model state
    x_r = np.asarray(self.ref.state, dtype=float).reshape(-1)
    yr = self.Cm.dot(x_r)
    CmAm = self.Cm.dot(self.Am)
    yr_dot = CmAm.dot(x_r)
    yr_ddot = CmAm.dot(self.Am).dot(x_r) + CmAm.dot(self.Bm) * rv

    # v_j = Abar^j lam ; second/first/third components used below
    v = [self.Abar_pows[j].dot(self.lam) for j in range(m + 1)]
    # row2(Xi): slot j (multiplying a_{n-j}) is -(Abar^{n-j} eta)_2, j=1..n
    xrow = np.array([-(self.Abar_pows[n - j].dot(self.eta))[1] for j in range(1, n + 1)])

    omega = np.hstack([[v[j][1] for j in range(m, -1, -1)], xrow])
    omega[m + 1] -= y  # -y goes with the a_{n-1} slot
    omega_bar = omega[1:]

    xi2 = self.xi[1]
    theta_bar_hat = self.theta_hat[1:]

    # Step 1
    z1 = y - yr
    alpha1_bar = -(self.c1 + self.d1) * z1 - xi2 - omega_bar.dot(theta_bar_hat) + yr_dot
    alpha1 = self.rho_hat * alpha1_bar
    z2 = omega[0] - alpha1
    # a_{n-1} estimate sits right after the b block
    dalpha1_dy = self.rho_hat * (-(self.c1 + self.d1) + self.theta_hat[m + 1])

    # Update laws (tuning functions, normalized for discrete-time stability,
    # sigma-modification against drift, derivative projection for rho_hat)
    n_s = 1.0 + self.nu * (omega.dot(omega) + z1**2 + z2**2)
    rho_dot = (-self.gamma_rho * self.sgn_bm * alpha1_bar * z1 / n_s
               - self.gamma_rho * self.sigma * (self.rho_hat - self.rho_hat0))
    lo, hi = self.rho_bounds
    if (self.rho_hat <= lo and rho_dot < 0.0) or (self.rho_hat >= hi and rho_dot > 0.0):
      rho_dot = 0.0
    tau2 = omega * z1 - dalpha1_dy * omega * z2
    tau2[0] -= alpha1 * z1
    theta_dot = self.gamma_mat.dot(tau2 / n_s - self.sigma * self.theta_hat)

    # Computable part of alpha1_dot (everything except the dalpha1/dy * y_dot route)
    xi2_dot = self.xi[2] - k2 * self.xi[0] + k2 * y
    omega_bar_f_dot = np.hstack([
      [v[j][2] - k2 * v[j][0] for j in range(m - 1, -1, -1)],
      [-(self.Abar_pows[n - j + 1].dot(self.eta))[1] - (y if j == 2 else 0.0)
       for j in range(1, n + 1)],
    ])
    beta = alpha1_bar * rho_dot + self.rho_hat * (
      (self.c1 + self.d1) * yr_dot + yr_ddot - xi2_dot
      - theta_dot[1:].dot(omega_bar) - theta_bar_hat.dot(omega_bar_f_dot))

    # Step 2 control law
    u = (-self.theta_hat[0] * z1
         - (self.c2 + self.d2 * dalpha1_dy**2) * z2
         - v[m][2] + k2 * v[m][0]
         + dalpha1_dy * (xi2 + omega.dot(self.theta_hat))
         + beta)

    return u, theta_dot, rho_dot, z1, z2

  def calc_u(self, yp, r):
    u, _, _, _, _ = self._signals(yp, r)
    u = np.array([[u]])
    if self.umin is not None and self.umax is not None:
      u = np.clip(u, self.umin, self.umax)
    return u

  def update(self, dt, yp, r, u):
    """
    Advance adaptive states, K-filters and the reference model by dt.
    Must be called after calc_u with the actually applied (saturated) u,
    before the next observation is used.

    The adaptation ODE is integrated with adapt_substeps sub-steps while the
    measurements and filter states are held: the update laws depend on the
    current parameter estimates through alpha1, z2 and dalpha1/dy, and a
    single Euler step of that coupled loop goes unstable for large signals
    even though the continuous-time design is stable.
    """
    h = dt / self.adapt_substeps
    lo, hi = self.rho_bounds
    for _ in range(self.adapt_substeps):
      _, theta_dot, rho_dot, _, _ = self._signals(yp, r)
      self.theta_hat = self.theta_hat + theta_dot * h
      # clip after the step too: the derivative projection alone lets one
      # Euler substep overshoot the bound (and possibly flip sgn(rho_hat))
      self.rho_hat = min(max(self.rho_hat + rho_dot * h, lo), hi)

    y = float(np.asarray(yp).flat[0])
    uv = float(np.asarray(u).flat[0])
    Ad, Bd_k, Bd_en = self._discretization(dt)
    self.xi = Ad.dot(self.xi) + Bd_k * y
    self.eta = Ad.dot(self.eta) + Bd_en * y
    self.lam = Ad.dot(self.lam) + Bd_en * uv

    self.ref.update(dt, r)
