from scipy.integrate import odeint

class LongitudialVechidle:

  def __init__(self, vehicleParam) -> None:
    self.vehicle_param = vehicleParam
  
  def update(self, dt, input):
    u = input
    