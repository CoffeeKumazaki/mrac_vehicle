import numpy as np

def calc_distance(pa, pb, pv): 
  u = np.array([pb[0] - pa[0], pb[1] - pa[1]])
  v = np.array([pv[0] - pa[0], pv[1] - pa[1]])
  if (np.linalg.norm(u) == 0.0) :
    L = 0.0
  else:
    L = np.cross(u, v) / np.linalg.norm(u)
  return L