import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import warnings
warnings.simplefilter('ignore')

from control.matlab import *
from mrac.parameter_estimator import current_tf 
from vehicle_params import *
from vehicle_dynamics import *
from mrac.parameter_estimator import *
from mrac.controller_designer import designed_state_space_eq

data_path = "../../data/output/no_init_param/adaptive_vehicle_vx_05to10_mvx_20_gain_10_theta.csv"
params = pd.read_csv(data_path, header=None, delimiter=' ')

intetval = 100

gif_name = data_path.split("/")[-1].split(".")[0]

print(gif_name)

vx = 10.0
lbd0 = [1, 1]

## Plant Model
plantParam = VehicleParam()
plantParam.mass = 1727.0
plantParam.cf = 94000.0
plantParam.cr = 94000.0
plantParam.lf = 1.17
plantParam.lr = 1.42
plantParam.Iz = 2867.0

A, B, C, D = linear_vehicle_model(plantParam, vx)
plant_sys = ss(A, B, C, D)

## Reference Model
modelParam = VehicleParam()
A, B, C, D = linear_vehicle_model_fb(modelParam, vx, -2.0, -0.05)
reference_sys = ss(A, B, C, D)

n, L, _ = designed_state_space_eq(reference_sys, lbd0)
lbd = np.hstack([[1.0], -L[0]])

def draw_frame(i, thetas, mode):

  param = thetas[i]
  dim = int((len(param)- 1 - 2) / 2.0)
  theta = list(param[1:])
  num, den = current_tf(plant_sys, theta, dim, lbd)
  est_tf = tf(num, den)
  plt.clf()
  if (mode == "pgmap"):
    #pzmap(est_tf)
    #pzmap(reference_sys)
    plt.xlim(-30, 5)
    plt.ylim(-20, 20)

    def draw_pgmap(system, color, size=50, label=""):
      zeros = zero(system)
      zeros_x = zeros.real
      zeros_y = zeros.imag

      poles = pole(system)
      poles_x = poles.real
      poles_y = poles.imag
      plt.scatter(zeros_x, zeros_y, marker='x', color=color, s=size, label=label+" zeros")
      plt.scatter(poles_x, poles_y, marker='o', facecolors='none', edgecolors=color, s=size, label=label+" poles")
      if label != "":
        plt.legend()

    draw_pgmap(est_tf, "blue", label="plant")
    draw_pgmap(reference_sys, "red", 100, label="reference")
    plt.axhline(y=0, color="gray")
    plt.axvline(x=0, color="gray")

  elif (mode == "bode"):
    bode(est_tf)
    bode(reference_sys)
  else:
    pass
  plt.title("t:{}".format(i) )

fig = plt.figure() #figure objectを取得

p = params[::intetval].to_numpy()
# p = params[15800:].to_numpy()

ani = animation.FuncAnimation(fig, draw_frame, fargs = (p,"pgmap"), interval = 100, frames = len(p))
ani.save(gif_name + "_pgmap.gif")

ani = animation.FuncAnimation(fig, draw_frame, fargs = (p,"bode"), interval = 100, frames = len(p))
ani.save(gif_name + "_bode.gif")