"""
Vehicle object.
@Author: Kristian Gibson
TODO: Bug-fixing, change remaining matlab placeholders
TODO: Comments + Sphynx Docs Structured Text
TODO: Replace vargin with parameters

Not ready for use yet.
"""

from numpy import disp
from scipy import integrate, randn
from scipy.linalg import sqrtm
from spatialmath.base.transforms2d import *
from spatialmath.base.vectors import *
from roboticstoolbox.robot.utility import numrows

class Vehicle:

    def __init__(self, x, x_hist, speedmax, dim, rdim, dt, v, odometry, verbose, driver, x0, options,
                 vhandle, vtrail):
        # state
        self._x = x                 # true state (x,y,theta)
        self._x_hist = x_hist       # x history

        # parameters
        self._speedmax = speedmax   # maximum speed
        self._dim = dim             # dimension of the world -dim -> +dim in x and y
        self._rdim = rdim           # dimension of the robot
        self._dt = dt               # sample interval
        self._v = v                 # odometry covariance
        self._odometry = odometry   # distance moved in last interval
        self._verbose = verbose
        self._driver = driver       # driver object
        self._x0 = x0               # initial state
        self._options = options
        self._vhandle = vhandle     # handle to vehicle graphics object
        self._vtrail = vtrail     # vehicle trail

        # TODO replace opt with further parameters
        """oplt.covar = []
        oplt.rdim = 0.2
        oplt.dt = 0.1
        oplt.x0 = zeros(3, 1)
        oplt.speedmax = 1
        oplt.vhandle = []

        [opt, args] = tb_optparse(opt, varargin)

        self._v = oplt.covar
        self._rdim = oplt.rdim
        self._dt = oplt.dt
        self._x0 = oplt.x0(:)
        assert (isvec(self._x0, 3), 'Initial configuration must be a 3-vector')
        self._speedmax = oplt.speedmax
        self._options = args  # unused options go back to the subclass
        self._vhandle = oplt.vhandle"""
        self._x_hist = []

        """if nargin > 1:
            self._x = x0(:)
        else:
            self._x = self._x0
        self._x_hist = []"""

        if any(self._driver):
            self._driver.init()

        self._vhandle = []

    @property
    def x(self):
        return self._x

    @property
    def x_hist(self):
        return self._x_hist

    @property
    def speedmax(self):
        return self._speedmax

    @property
    def dim(self):
        return self._dim

    @property
    def rdim(self):
        return self._rdim

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
    def options(self):
        return self._options

    @property
    def vhandle(self):
        return self._vhandle

    @property
    def vtrail(self):
        return self._vtrail

    #TODO: update from matlab
    def path(self, t, u, y0):
        if len(t) == 1:
            tt = [0 t]
        else:
            tt = t

        """if nargin < 4:
            y0 = [0 0 0]
        out = ode45( @(t,y) self._deriv(t, y, u), tt, y0)

        y = np.transpose(out.y)
        if nargout == 0:
            plt.plot(y[1], y[2])
            plt.xlabel('X')
            plt.ylabel('Y')
        else:
            yy = y
            if len(t) == 1:
                # if scalar time given, just return final state
                yy = yy(end,:)
        """
        return yy

    def add_driver(self, driver):
        self._driver = driver

    def update(self, u):
        xp = self._x # previous state
        self._x[0] = self._x[0] + u[0]*self._dt*np.cos(self._x[2])
        self._x[1] = self._x[1] + u[0]*self._dt*np.sin(self._x[2])
        self._x[2] = self._x[2] + u[0]*self._dt/self._L * u[1]
        odo = [np.norm(self._x[0:1]-xp[0:1]), self._x[2]-xp[2]]
        self._odometry = odo

        self._x_hist = [self._x_hist, np.transpose(self._x)]   # maintain history
        return odo

    def step(self, varargin):
        u = self._control(varargin[:])

        # compute the true odometry and update the state
        odo = self._update(u)

        # add noise to the odometry
        if any(self._v):
            odo = self._odometry + randn(1,2)*sqrtm(self._v)
        return odo

    def control(self, speed, steer):
        u = None
        nargin = None
        if nargin < 2:
            # if no explicit demand, and a driver is attached, use
            # it to provide demand
            if any(self._driver):
                [speed, steer] = self._driver.demand()
            else:
                # no demand, do something safe
                speed = 0
                steer = 0

        # clip the speed
        if not any(self._speedmax):
            u[0] = speed
        else:
            u[0] = np.min(self._speedmax, np.max(-self._speedmax, speed))

        # clip the steering angle
        #Todo: remove isprop and replace with parameter checks
        if isprop(self, 'steermax') and any(self._steermax):
            u[1] = np.max(-self._steermax, np.min(self._steermax, steer))
        else:
            u[1] = steer
        return u

    def run(self, nsteps):
        nargin = None
        nargout = None
        if nargin < 2:
            nsteps = 1000
        if any(self._driver):
            self._driver.init()
        #self._clear()
        if any(self._driver):
            self._driver.plot()

        self._plot()
        for i in range(nsteps):
            self._step()
            if nargout == 0:
                # if no output arguments then plot each step
                self._plot()
        p = self._x_hist
        return p

    def run2(self, T, x0, speed, steer):
        self._init(x0)

        for i in range(T/self._dt):
            self._update([speed, steer])
        p = self._x_hist
        return p

    def plot(self, varargin):
        if not any(self._vhandle):
            self._vhandle = Vehicle.plotv(self._x, varargin[:])

        if any(varargin) and isinstance(varargin[1], int):
            # V.plot(X)
            pos = varargin[1] # use passed value
        else:
            # V.plot()
            pos = self._x    # use current state

        # animate it
        Vehicle.plotv(self._vhandle, pos)

    def plot_xy(self, varargin):
        nargout = None
        xyt = self._x_hist
        if nargout == 0:
            plt.plot(xyt[0], xyt[1], varargin[:])
        else:
            out = xyt
        return xyt

    def verbosity(self, v):
        self._verbose = v

    def display(self, nav):
        #loose = strcmp( get(0, 'FormatSpacing'), 'loose')
        if loose:
            disp(' ')
        disp([inputname[0], ' = '])
        disp(nav)

    def char(self, veh):
        s = ('Superclass: Vehicle' + "\nmax speed: " + self._speedmax +  " dT: " + self._speedmax + "nhist" + self._dt
            + " " + numrows(self._x_hist))
        if any(self._v):
            s += "    V=(" + self._v(1,1) + ", " + self._v(2,2) + ")"

        s += "    configuration: x: " + self._x + " y: " + self._x + " theta: " + self._x
        if any(self._driver):
            s += '    driven by::' + self._driver

        return s

    #TODO Change vargins to params
    def plotv(self, varargin):
        if isinstance(varargin[1], dict):
            plt.plot(varargin[1], 'handle', varargin[1]) #Change to vehicle_plot
        else:
            h = plt.plot(varargin[1], 'fillcolor', 'b', 'alpha', 0.5)
        return h