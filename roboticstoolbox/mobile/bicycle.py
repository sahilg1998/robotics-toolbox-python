"""
Python Bicycle Car-like vehicle class
@Author: Kristian Gibson
TODO: Comments + Sphynx Docs Structured Text
TODO: Bug-fix, testing
"""
from numpy import disp
from scipy import integrate
from scipy import linalg
import matplotlib.pyplot as plt
from spatialmath.base.transforms2d import *
from spatialmath.base.vectors import *
from roboticstoolbox.mobile.vehicle import Vehicle


class Bicycle(Vehicle):
    def __init__(self, steer_max=None, accel_max=None, covar=0, speed_max=1, l=1, x0=np.array([0, 0, 0]),
                 dt=0.1, r_dim=0.2, verbose=None):
        super().__init__(self, covar, speed_max, l, x0, dt, r_dim, steer_max, verbose)
        self._x = np.zeros(3, 1)
        self._l = 1
        self._steer_max = 0.5
        self._accel_max = np.inf

        if covar is not None:
            self._v = covar
        if speed_max is not None:
            self._speed_max = speed_max
        if l is not None:
            self._l = l
        if x0 is not None:
            self._x0 = x0
            # TODO: Add assert
        if dt is not None:
            self._dt = dt
        if r_dim is not None:
            self._r_dim = r_dim
        if verbose is not None:
            self._verbose = verbose
        if steer_max is not None:
            self._steer_max = steer_max
        if accel_max is not None:
            self._accel_max = accel_max

        self._v_prev = 0
        self._x = self._x0

    @property
    def accel_max(self):
        return self._accel_max

    @property
    def x(self):
        return self._v_prev

    def f(self, x=None, odo=None, w=None):
        if w is None:
            w = np.array([0, 0])

        dd = odo[0] + w[0]
        dth = odo[1] + w[1]
        thp = x[:, 2]
        x_next = x + [dd * np.cos(thp), dd * np.sin(thp), np.ones(np.size(x, 0)*dth)]

        return x_next

    
