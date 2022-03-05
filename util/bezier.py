import numpy as np
from math import *

# Bezier Class representing a CUBIC bezier defined by four
# control points.
# 
# at(t):            gets a point on the curve at t
# distance2(pt)      returns the closest distance^2 of
#                   pt and the curve
# closest(pt)       returns the point on the curve
#                   which is closest to pt
# maxes(pt)         plots the curve using matplotlib
class Bezier(object):
    exp3 = np.array([[3, 3], [2, 2], [1, 1], [0, 0]], dtype=np.float32)
    exp3_1 = np.array([[[3, 3], [2, 2], [1, 1], [0, 0]]], dtype=np.float32)
    exp4 = np.array([[4], [3], [2], [1], [0]], dtype=np.float32)
    exp2 = np.array([[2], [1], [0]], dtype=np.float32)
    boundaries = np.array([0, 1], dtype=np.float32)

    # Initialize the curve by assigning the control points.
    # Then create the coefficients.
    def __init__(self, points):
        assert isinstance(points, np.ndarray)
        assert points.dtype == np.float32
        self.points = points
        self.create_coefficients()
    
    # Create the coefficients of the bezier equation, bringing
    # the bezier in the form:
    # f(t) = a * t^3 + b * t^2 + c * t^1 + d
    #
    # The coefficients have the same dimensions as the control
    # points.
    def create_coefficients(self):
        points = self.points
        a = - points[0] + 3*points[1] - 3*points[2] + points[3]
        b = 3*points[0] - 6*points[1] + 3*points[2]
        c = -3*points[0] + 3*points[1]
        d = points[0]
        self.coeffs = np.stack([a, b, c, d]).reshape(-1, 4, 2)

    # Return a point on the curve at the parameter t.
    def at(self, t):
        if type(t) != np.ndarray:
            t = np.array(t)
        pts = self.coeffs * np.power(t, self.exp3_1)
        return np.sum(pts, axis = 1)

    # Return the closest DISTANCE (squared) between the point pt
    # and the curve.
    def distance2(self, pt):
        points, angles, distances, index = self.measure_distance(pt)
        return distances[index], angles[index]

    # Return the closest POINT between the point pt
    # and the curve.
    def closest(self, pt):
        points, angles, distances, index = self.measure_distance(pt)
        return points[index], angles[index], sqrt(distances[index])

    def angle(self, t):
      coeffs = self.coeffs

      a = 3*coeffs[0][0]
      b = 2*coeffs[0][1]
      c = 1*coeffs[0][2]
      dcoeffs = np.stack([a, b, c])
      deviations = np.sum(dcoeffs * np.power(t, self.exp2), axis=1)
      return np.arctan2(deviations[:, 1], deviations[:, 0])


    # Measure the distance^2 and closest point on the curve of 
    # the point pt and the curve. This is done in a few steps:
    # 1     Define the distance^2 depending on the pt. I am 
    #       using the squared distance because it is sufficient
    #       for comparing distances and doesn't have the over-
    #       head of an additional root operation.
    #       D(t) = (f(t) - pt)^2
    # 2     Get the roots of D'(t). These are the extremes of 
    #       D(t) and contain the closest points on the unclipped
    #       curve. Only keep the minima by checking if
    #       D''(roots) > 0 and discard imaginary roots.
    # 3     Calculate the distances of the pt to the minima as
    #       well as the start and end of the curve and return
    #       the index of the shortest distance.
    #
    # This desmos graph is a helpful visualization.
    # https://www.desmos.com/calculator/ktglugn1ya
    def measure_distance(self, pt):
        coeffs = self.coeffs

        # These are the coefficients of the derivatives d/dx and d/(d/dx).
        da = 6*np.sum(coeffs[0][0]*coeffs[0][0])
        db = 10*np.sum(coeffs[0][0]*coeffs[0][1])
        dc = 4*(np.sum(coeffs[0][1]*coeffs[0][1]) + 2*np.sum(coeffs[0][0]*coeffs[0][2]))
        dd = 6*(np.sum(coeffs[0][0]*(coeffs[0][3]-pt)) + np.sum(coeffs[0][1]*coeffs[0][2]))
        de = 2*(np.sum(coeffs[0][2]*coeffs[0][2])) + 4*np.sum(coeffs[0][1]*(coeffs[0][3]-pt))
        df = 2*np.sum(coeffs[0][2]*(coeffs[0][3]-pt))

        dda = 5*da
        ddb = 4*db
        ddc = 3*dc
        ddd = 2*dd
        dde = de
        dcoeffs = np.stack([da, db, dc, dd, de, df])
        ddcoeffs = np.stack([dda, ddb, ddc, ddd, dde]).reshape(-1, 1)
        
        # Calculate the real extremes, by getting the roots of the first
        # derivativ of the distance function.
        extrema = Bezier.np_real_roots(dcoeffs)
        # Remove the roots which are out of bounds of the clipped range [0, 1].
        # [future reference] https://stackoverflow.com/questions/47100903/deleting-every-3rd-element-of-a-tensor-in-tensorflow
        dd_clip = (np.sum(ddcoeffs * np.power(extrema, self.exp4)) >= 0) & (extrema > 0) & (extrema < 1)
        minima = extrema[dd_clip]

        # Add the start and end position as possible positions.
        potentials = np.concatenate((minima, self.boundaries))

        # Calculate the points at the possible parameters t and 
        # get the index of the closest
        points = self.at(potentials.reshape(-1, 1, 1))
        angles = self.angle(potentials.reshape(-1, 1, 1))
        distances = np.sum(np.square(points - pt), axis = 1)
        index = np.argmin(distances)

        return points, angles, distances, index

    @staticmethod
    def np_real_roots(coefficients, EPSILON=1e-6):
        r = np.roots(coefficients)
        return r.real[abs(r.imag) < EPSILON]
