class VehicleParam():
  
  def __init__(self):
    self.mass = 1650.0 # mass [kg]
    self.Iz = 2900.0
    self.cf = 100000.0
    self.cr = 200000.0
    self.lf = 1.1 # 
    self.lr = 1.6
    self.zeta = 0.7
    self.omega_n = 17.5

  def __repr__(self) -> str:
    return f'''Vehicle Parameter
      mass:    {self.mass}
      Iz:      {self.Iz}
      cf:      {self.cf}
      cr:      {self.cr}
      lf:      {self.lf}
      lr:      {self.lr}
      zeta:    {self.zeta}
      omega_n: {self.omega_n}
    '''

  def __str__(self) -> str:
      return self.__repr__()