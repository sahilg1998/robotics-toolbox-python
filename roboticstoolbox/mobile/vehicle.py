"""
Python Vehicle
@Author: Kristian Gibson
TODO: Comments + Sphynx Docs Structured Text
TODO: Bug-fix, testing

Not ready for use yet.
"""
from numpy import disp
from scipy import integrate
from scipy import linalg
import matplotlib.pyplot as plt
from roboticstoolbox.mobile import *
from spatialmath.base.transforms2d import *
from spatialmath.base.vectors import *


class Vehicle:
    def __init__(self, covar=None, speed_max=None, l=None, x0=None, dt=None, r_dim=None,
                 steer_max=None, verbose=None):
        self._covar = np.array([])
        self._r_dim = 0.2
        self._dt = 0.1
        self._x0 = np.zeros(3, 1)
        self._x = None
        self._speed_max = 1
        self._v_handle = np.array([])
        self._driver = None
        self._odometry = None

        # TODO: Do we even need these if statements?
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

        self._x_hist = np.array([])

    @property
    def l(self):
        return self._l

    @property
    def steer_max(self):
        return self._steer_max

    @property
    def x(self):
        return self._x

    @property
    def x_hist(self):
        return self._x_hist

    @property
    def speed_max(self):
        return self._speed_max

    @property
    def dim(self):
        return self._dim

    @property
    def r_dim(self):
        return self._r_dim

    @property
    def dt(self):
        return self._dt

    @property
    def v(self):
        return self._v

    @property
    def odometry(self):
        return self._odometry

    @property
    def verbose(self):
        return self._verbose

    @property
    def driver(self):
        return self._driver

    @property
    def x0(self):
        return self._x0

    @property
    def v_handle(self):
        return self._v_handle

    @property
    def v_trail(self):
        return self._v_trail

    @property
    def driver(self):
        return self._driver

    # Example
    def init(self, x0=None):
        if x0 is not None:
            self._x = x0
        else:
            self._x = self._x0

        self._x_hist = np.array([])

        if self._driver is not None:
            self._driver.init()  # TODO: make this work?

        self._v_handle = np.array([])

    def path(self, t=None, u=None, y0=None):  # TODO: Might be the source of some errors
        tt = None
        yy = None

        if len(t) == 1:
            tt = np.array([0, t[-1]])
        else:
            tt = t

        if y0 is None:
            y0 = np.array([0, 0, 0])

        ode_out = integrate.solve_ivp(self.deriv(t, None, u), [tt[0], tt[-1]], y0, t_eval=tt, method="RK45")
        y = np.transpose(ode_out.y)

        if t is None:
            plt.plot(y[:][0], y[:][2])
            plt.xlabel('X')
            plt.ylabel('Y')
        else:
            yy = y
            if len(t) == 1:
                yy = yy[-1][:]

        return yy

    # This function is overridden by the child class
    def deriv(self, t, y=None, u=None):  # TODO: I have no idea where Y comes from, here!
        return y

    def add_driver(self, driver):
        self._driver = driver
        driver._veh = self

    def update(self, u):
        xp = self._x
        self._x[0] = self._x[0] + u[0] * self._dt * np.cos(self._x[2])
        self._x[1] = self._x[1] + u[0] * self._dt * np.sin(self._x[2])
        self._x[2] = self._x[2] + u[0] * self._dt / self._l * u[1]
        odo = np.array([col_norm(self._x[0:2] - xp[0:2], self._x[2] - xp[2])])  # TODO: Right indexing?
        self._odometry = odo

        self._x_hist = np.concatenate(self._x_hist, np.transpose(self._x))
        return odo

    def step(self, speed=None, steer=None):
        u = self.control(speed, steer)
        odo = self.update(u)

        if self._v is not None:
            odo = self._odometry + np.random.rand(1, 2) * linalg.sqrtm(self._v)  # TODO: linalg imported?

        return odo

    def control(self, speed=None, steer=None):
        u = np.zeros(2)
        if speed is None and steer is None:
            if self._driver is not None:
                speed, steep = self._driver.demand()
            else:
                speed = 0
                steer = 0

        if self._speed_max is None:
            u[0] = speed
        else:
            u[0] = np.minimum(self._speed_max, np.maximum(-self._speed_max, speed))

        if self._steer_max is not None:
            u[1] = np.maximum(-self._steer_max, np.minimum(self._steer_max, steer))
        else:
            u[1] = steer

        return u

    def run(self, n_steps=None):
        if n_steps is None:
            n_steps = 1000
        if self._driver is not None:
            self._driver.init()
        if self._driver is not None:
            self._driver.plot()

        self._plot()
        for i in range(0, n_steps):
            self.step()
            # TODO: There's a nargout here... is this really needed or can it be done differently?

        p = self._x_hist
        return p

    def run_2(self, t, x0, speed, steer):
        self.init(x0)

        for i in range(0, (t/self._dt)):
            self.update(np.array([speed, steer]))

        p = self._x_hist
        return p

    def plot(self):
        # TODO: Add vargin arguments. There's more here.
        if self._v_handle is None:
            self._v_handle = plot_v(self._x)

        pos = self._x
        plot_v(self._v_handle, pos)

    def plot_xy(self):
        # TODO: this also has some vargin
        xyt = self._x_hist
        plt.plot(xyt[0, :], xyt[1, :])

    def verbiosity(self, v):
        self._verbose = v

    def display(self, nav):
        # write later idk what they want
        disp("display function doesn't currently do anything")

    def char(self):
        s = 'Superclass: Vehicle' + \
            "\nMax speed=" + self._speed_max + \
            "\ndt=" + self._dt + \
            "\nn_hist=" + np.shape(self._x_hist)[0]

        if self._v is not None:
            s = s + "\nV=" + self._v[0, 0] + ", " + self._v[1, 1] + ")"

        # Todo: this one should show x, y, theta
        s = s + "Configuration:" + self._x

        if self._driver is not None:
            s = s + "\nDriven by: " + self._driver  # TODO: this might break

        return s


def plot_v(handle=None, pose_x=None):
    # TODO add vargin stuff
    if handle is not None:
        plot_vehicle(pose_x, handle)
    else:
        handle = None
        fillcolor = 'b'
        alpha = 0.5
        h = plot_vehicle(pose_x, handle, fillcolor, alpha)
        return h


def col_norm(x):
    y = np.array([])
    if x.ndim > 1:
        x = np.column_stack(x)
        for vector in x:
            y = np.append(y, np.linalg.norm(vector))
    else:
        y = np.linalg.norm(x)
    return y
