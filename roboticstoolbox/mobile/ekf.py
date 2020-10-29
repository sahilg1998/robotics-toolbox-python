"""
Python EKF Planner
@Author: Kristian Gibson
TODO: Comments + Sphynx Docs Structured Text
TODO: Bug-fix, testing, change remaining matlab placeholders
TODO: Replace vargin with parameters

Not ready for use yet.
"""

from numpy import disp
from scipy import integrate, randn
from scipy.linalg import sqrtm
from spatialmath.base import Animate
from spatialmath.base.transforms2d import *
from spatialmath.base.vectors import *
from roboticstoolbox.robot.utility import numrows

class EKF:
    def __init__(self, robot, v_est, p0, x_est=None, p_est=None, landmarks=None, w_est=None, sensor=None,
                 est_vehicle=None, est_ekf_map=None, joseph=None, verbose=None, keep_history=None,
                 ekf_map=None, history=None, dim=None):
        self._x_est = x_est           #  estimated state
        self._p_est = p_est           #  estimated covariance
        self._landmarks = landmarks           #  ekf_map state
        self._v_est = v_est           #  estimate of covariance V
        self._w_est = w_est           #  estimate of covariance W
        self._robot = robot           #  reference to the robot vehicle
        self._sensor = sensor          #  reference to the sensor
        self._est_vehicle = est_vehicle      #  flag: estimating vehicle location
        self._est_ekf_map = est_ekf_map          #  flag: estimating ekf_map
        self._joseph = joseph          #  flag: use Joseph form to compute p
        self._verbose = verbose
        self._keep_history = keep_history     #  keep history
        self._p0 = p0              #  passed initial covariance
        self._ekf_map = ekf_map             #  passed ekf_map
        self._history = history
        self._dim = dim          #  robot workspace dimensions

        self._robot.init()

        #  clear the history
        self._history = []

        if not any(self._v_est):
            #  perfect vehicle case
            self._est_vehicle = False
            self._x_est = []
            self._p_est = []
        else:
            #  noisy odometry case
            self._x_est = self._robot.x   #  column vector
            self._p_est = self._p0
            self._est_vehicle = True

        if any(self._sensor):
            self._landmarks = None*np.zeros(2, self._sensor.ekf_map.nlandmarks)

        #TODO Replace with parameters
        """opt.history = True
        opt.joseph = True
        opt.dim = []

        [opt,args] = tb_optparse(opt, varargin)

        switch len(args)
            case 0
                #  Deadreckoning:
                #     E = EKF(VEHICLE, V_EST, p0, OPTIONS)
                sensor = [] w_est = [] ekf_map = []
                self._est_vehicle = True
            case 3
                #  Using a ekf_map:
                #     E = EKF(VEHICLE, V_EST, p0, SENSOR, W_EST, MAP, OPTIONS)
                #  Estimating a ekf_map:
                #     E = EKF(VEHICLE,[], [], SENSOR, W_EST, [], OPTIONS)
                #  Full SLAM:
                #     E = EKF(VEHICLE, V_EST, p0, SENSOR, W_EST, [], OPTIONS)

                [sensor, w_est, ekf_map] = deal(args[i])
                if not any(self_map)
                    self._est_ekf_map = True

                if any(v_est)
                    self._est_vehicle = True
            otherwise
                Error('RTB:EKF:badarg', 'incorrect number of non-option arguments')"""

        #  check types for passed objects
        if any(self_map) and not isinstance(self_map, 'LandmarkMap'):
            Error('RTB:EKF:badarg\nexpecting LandmarkMap object')

        if any(sensor) and not isinstance(sensor, 'Sensor'):
            Error('RTB:EKF:badarg\nexpecting Sensor object')

        if not isinstance(robot, "Vehicle"):
            Error('RTB:EKF:badarg\nexpecting Vehicle object')

        self._init()

    
    @property
    def x_est(self):
        return self._x_est
    
    @property
    def p_est(self):
        return self._p_est
    
    @property
    def landmarks(self):
        return self._landmarks
    
    @property
    def v_est(self):
        return self._v_est

    @property
    def w_est(self):
        return self._w_est
    
    @property
    def robot(self):
        return self._robot
    
    @property
    def sensor(self):
        return self._sensor
    
    @property
    def est_vehicle(self):
        return self._est_vehicle
    
    @property
    def est_ekf_map(self):
        return self._est_ekf_map
    
    @property
    def joseph(self):
        return self._joseph
    
    @property
    def verbose(self):
        return self._verbose
    
    @property
    def keep_history(self):
        return self._keep_history
    
    @property
    def p0(self):
        return self._p0
    
    @property
    def ekf_map(self):
        return self._ekf_map
    
    @property
    def history(self):
        return self._history
    
    @property
    def dim(self):
        return self._dim
    
    def run(self, n, varargin):
        """opt.plt.plot = True
        opt.movie = []
        opt = tb_optparse(opt, varargin)"""

        self._init()

        if opt.plt.plot:
            if any(self._sensor):
                self._sensor.ekf_map.plt.plot()
            elif any(self._dim):
                if len(self._dim) == 1:
                    d = self._dim
                    plt.axis([-d, d, -d, d])
                if len(self._dim) == 2:
                    w = self._dim[1], h = self._dim(2)
                    plt.axis([-w, w, -h, h])
                if len(self._dim) == 4:
                    plt.axis(self._dim)

                #set(gca, 'ALimMode', 'manual')
            else:
                opt.plt.plot = False

            plt.xlabel('X')
        plt.ylabel('Y')

        anim = Animate(opt.movie)
        for k in range(n):

            if opt.plt.plot:
                self._robot.plt.plot()

            self._step(opt)
            anim.add()
        anim.close()

    def get_xy(self):
        if self._est_vehicle:
            xyt = np.zeros(len(self._history), 3)
            for i in range(len(self._history)):
                h = self._history[i]
                xyt[i,:] = np.invert(h.x_est[1:3])
        else:
            xyt = []
        return xyt

    def plot_xy(self, varargin):
        xyt=self._get_xy()
        plt.plot(xyt[0], xyt[1], varargin[i])
        plt.plot(xyt(1,1), xyt(1,2), 'ko', 'MarkerSize', 8, 'LineWidth', 2)

    def plot_Error(self, varargin):
        #opt.color = 'r'
        #opt.confidence = 0.95
        #opt.nplt.plots = 3

        #[opt,args] = tb_optparse(opt, varargin)

        if self._est_vehicle:
            err = np.zeros(len(self._history), 3)
            for i in range(len(self._history)):
                h = self._history[i]
                #  Error is True - estimated
                err[i,:] = self._robot.x_hist[i,:] - np.invert(h.x_est[1:3])
                err[i,2] = angdiff(err[i,2])
                P = diag(h.P)
                pxy[i,:] = sqrt( chi2inv_rtb(opt.confidence, 2)*P[1:3] )

            if nargout == 0:         
                t = 1:numrows(pxy)
                t = [t t(:-1:1)]'
                plt.subplot(opt.nplt.plots*100+11)
                
                if opt.confidence:
                    edge = [pxy[0] -pxy(:-1:1,1)]
                    h = patch(t, edge ,opt.color)
                    set(h, 'EdgeColor', 'none', 'FaceAlpha', 0.3)

                plt.plot(err[0], args[i])
                plt.ylabel('x Error')
                plt.subplot(opt.nplt.plots*100+12)
                
                if opt.confidence:
                    edge = [pxy[1] -pxy(:-1:1,2)]
                    h = patch(t, edge, opt.color)
                    set(h, 'EdgeColor', 'none', 'FaceAlpha', 0.3)

                
                plt.plot(err[1], args[i]):
                plt.ylabel('y Error')
                plt.subplot(opt.nplt.plots*100+13)
                
                if opt.confidence:
                    edge = [pxy[2] -pxy(:-1:1,3)]
                    h = patch(t, edge, opt.color)
                    set(h, 'EdgeColor', 'none', 'FaceAlpha', 0.3)

                plt.plot(err[2], args[i])
                
                plt.xlabel('Time step')
                plt.ylabel('\theta Error')

                if opt.nplt.plots > 3
                    plt.subplot(opt.nplt.plots*100+14)

            else:
                out = pxy

    def get_ekf_map(self, varargin):
        xy = []
        for i in range(np.numcols(self._landmarks)):
            n = self._landmarks(1,i)
            if n is None:
                #  this landmark never observed
                xy = [xy, [None, None]]
                continue

            #  n is an index into the *landmark* part of the state
            #  vector, we need to offset it to account for the vehicle
            #  state if we are estimating vehicle as well
            if self._est_vehicle:
                n = n + 3

            xf = self._x_est[n][n+1]
            xy = [xy, xf]
        return xy

    def plot_ekf_map(self, varargin)
    #  TODO:  some option to plt.plot ekf_map evolution, layered ellipses
        opt = None
        #opt.confidence = 0.95
        #[opt,args] = tb_optparse(opt, varargin)

        xy = []
        for i in range(np.numcols(self._landmarks)):
            n = self._landmarks(1,i)
            if n is None:
                #  this landmark never observed
                xy = [xy, [None, None]]
                continue
            if self._est_vehicle:
                n = n + 3

            xf = self._x_est[n][n+1]
            P = self._p_est[[n][n+1],[n][n+1]]
            #  TODO reinstate the interval landmark
            # plt.plot_ellipse(xf, P, interval, 0, [], varargin[i])
            plt.plot_ellipse( P * chi2inv_rtb(opt.confidence, 2), xf, args[i])
            plt.plot(xf(1), xf(2), 'k.', 'MarkerSize', 10)

    def get_P(self, k):
        P = np.zeros(len(self._history),1)
        for i in range(len(self._history)):
            P[i] = sqrt(np.det(self._history[i].P))
        return P

    def plot_P(self, varargin):
        p = self._get_P()

        plt.plot(p, varargin[i])
        plt.xlabel('Time step')
        plt.ylabel('(det P)^{0.5}')

    def show_P(self, k):
        if nargin < 2:
            k = len(self._history)

        z = np.log10(abs(self._history(k).P))
        mn = min(z(not np.isinf(z)))
        z(np.isinf(z)) = mn
        cekf_map = np.flip( gray(256), 1)
        # colorekf_map(parula)
        colorekf_map(np.flipud(bone))
        c = gray
        c = [np.ones(numrows(c),1) 1-c[:,1:2]]
        colorekf_map(c)

        plt.imshow(z, 'CDataMapping', 'scaled')
        plt.xlabel('state')
        plt.ylabel('state')

        c = colorbar()
        c.Label.String = 'log covariance'

    def plot_ellipse(self, varargin):
        opt.interval = round(len(self._history)/20)
        opt.confidence = 0.95

        [opt,args] = tb_optparse(opt, varargin)
        
        for i in range(1:opt.interval:len(self._history)):
            h = self._history[i]
            # plt.plot_ellipse(h.x_est(1:2), h.P(1:2,1:2), 1, 0, [], varargin[i])
            plt.plot_ellipse(h.P(1:2,1:2) * chi2inv_rtb(opt.confidence, 2), h.x_est(1:2), args[i])

    def display(self):
        loose = strcmp( get(0, 'FormatSpacing'), 'loose')
        if loose:
            disp(' ')

        disp([inputname(1), ' = '])
        disp( char(self) )
     #  display()

    def s = char(self):
        s = sprintf('EKF object: # d states', len(self._x_est))
        e = ''
        if self._est_vehicle:
            e = [e 'Vehicle ']

        if self._est_ekf_map:
            e = [e 'Map ']

        s = char(s, ['  estimating: ' e])
        if any(self._robot):
            s = char(s, char(self._robot))

        if any(self._sensor):
            s = char(s, char(self._sensor))

        s = char(s, ['w_est:  ' mat2str(self._w_est, 3)] )
        s = char(s, ['v_est:  ' mat2str(self._v_est, 3)] )

    # TODO: Make the following methods private.
    def x_est = step(self, opt):
        odo = self._robot.step()

        if self._est_vehicle:
            xv_est = self._x_est[1:3]
            xm_est = self._x_est(4:)
            Pvv_est = self._p_est(1:3,1:3)
            Pmm_est = self._p_est(4:,4:)
            Pvm_est = self._p_est(1:3,4:)
        else:
            xm_est = self._x_est
            # Pvv_est = self._p_est
            Pmm_est = self._p_est


        if self._est_vehicle:
            xv_pred = self._robot.f(xv_est', odo)'

            Fx = self._robot.Fx(xv_est, odo)
            Fv = self._robot.Fv(xv_est, odo)
            Pvv_pred = Fx*Pvv_est*Fx' + Fv*self._v_est*Fv'
        else:
            xv_pred = self._robot.x


        if self._est_ekf_map:
            if self._est_vehicle:
                Pvm_pred = Fx*Pvm_est

            Pmm_pred = Pmm_est
            xm_pred = xm_est

        if self._est_vehicle and not self._est_ekf_map:
            x_pred = xv_pred
            P_pred =  Pvv_pred
        elif not self._est_vehicle and self._est_ekf_map:
            x_pred = xm_pred
            P_pred = Pmm_pred
        elif self._est_vehicle and self._est_ekf_map:
            x_pred = [xv_pred xm_pred]
            P_pred = [ Pvv_pred Pvm_pred np.inverse(Pvm_pred) Pmm_pred]

        doUpdatePhase = False

        # disp('x_pred:') x_pred'
            # sensorReading = False
        if any(self._sensor)
            #  read the sensor
            [z,js] = self._sensor.reading()

            #  test if the sensor has returned a reading at this time interval
            sensorReading = js > 0


        if sensorReading
            #  here for MBL, MM, SLAM

            #  compute the innovation
            z_pred = self._sensor.h(xv_pred', js)'
            innov(1) = z(1) - z_pred(1)
            innov(2) = angdiff(z(2), z_pred(2))

            if self._est_ekf_map
                #  the ekf_map is estimated MM or SLAM case
                if self._seenBefore(js)

                    #  get previous estimate of its state
                    jx = self._landmarks(1,js)
                    xf = xm_pred(jx:jx+1)

                    #  compute Jacobian for this particular landmark
                    Hx_k = self._sensor.Hp(xv_pred', xf)
                    #  create the Jacobian for all landmarks
                    Hx = np.zeros(2, len(xm_pred))
                    Hx(:,jx:jx+1) = Hx_k

                    Hw = self._sensor.Hw(xv_pred, xf)

                    if self._est_vehicle
                        #  concatenate Hx for for vehicle and ekf_map
                        Hxv = self._sensor.Hx(xv_pred', xf)
                        Hx = [Hxv Hx]

                    doUpdatePhase = True

    #                 if mod(i, 40) == 0
    #                     plt.plot_ellipse(x_est(jx:jx+1), p_est(jx:jx+1,jx:jx+1), 5)
    #
                else
                    #  get the exted state
                    [x_pred, P_pred] = self._extMap(P_pred, xv_pred, xm_pred, z, js)
                    doUpdatePhase = False

            else
                #  the ekf_map is given, MBL case
                Hx = self._sensor.Hx(xv_pred', js)
                Hw = self._sensor.Hw(xv_pred', js)
                doUpdatePhase = True



            #  doUpdatePhase flag indicates whether or not to do
            #  the update phase of the filter
            #
            #   DR                        always False
            #   ekf_map-based localization    if sensor reading
            #   ekf_map creation              if sensor reading & not first
            #                               sighting
            #   SLAM                      if sighting of a previously
            #                               seen landmark

        if doUpdatePhase:
            # disp('do update\n')
            # #  we have innovation, update state and covariance
            #  compute x_est and p_est

            #  compute innovation covariance
            S = Hx*P_pred*Hx' + Hw*self._w_est*Hw'

            #  compute the Kalman gain
            K = P_pred*Hx' / S

            #  update the state vector
            x_est = x_pred + K*innov'

            if self._est_vehicle:
                #  wrap heading state for a vehicle
                x_est(3) = angdiff(x_est(3))


            #  update the covariance
            if self._joseph:
                #  we use the Joseph form
                I = np.eye(size(P_pred))
                p_est = (I-K*Hx)*P_pred*np.inverse(I-K*Hx) + K*self._w_est*np.inverse(K)
        else:
                p_est = P_pred - K*S*np.inverse(K)


            #  enforce P to be symmetric
            p_est = 0.5*(p_est+np.inverse(p_est))
        else:
            x_est = x_pred
            p_est = P_pred
            innov = []
            S = []
            K = []

        self._x_est = x_est
        self._p_est = p_est

        if self._keep_history:
            hist.x_est = x_est
            hist.odo = odo
            hist.P = p_est
            hist.innov = innov
            hist.S = S
            hist.K = K
            self._history = [self._history hist]

    def seenBefore(self, jf):
        if not isnan(self._landmarks(1,jf)):
            if self._verbose
                disp('landmark # d seen # d times before, state_idx=# d\n' jf, self._landmarks(2,jf), self._landmarks(1,jf))
            self._landmarks(2,jf) = self._landmarks(2,jf)+1
            s = True
        else
            s = False
        return s

    def extMap(self, P, xv, xm, z, jf):
        if self._verbose:
            disp('landmark # d first sighted\n', jf)

        xf = self._sensor.g(xv, z)

        if self._est_vehicle:
            x_ext = [xv xm xf]
        else:
            x_ext = [xm xf]

        Gz = self._sensor.Gz(xv, z)

        if self._est_vehicle:
            Gx = self._sensor.Gx(xv, z)
            n = len(self._x_est)
            M = [np.eye(n) np.zeros(n,2) Gx np.zeros(2,n-3) Gz]
            P_ext = M*blkdiag(P, self._w_est)*M'
        else:
            P_ext = blkdiag(P, Gz*self._w_est*Gz')

        self._landmarks(1,jf) = len(xm)+1
        self._landmarks(2,jf) = 1

        if self._verbose:
            disp('exted state vector\n')
        return x_ext, P_ext


def chi2inv_rtb(confidence, n):
    assert(n == 2, 'chi2inv_rtb: only valid for 2DOF')

    c = np.linspace(0,1,101)
    x = [0.000000 0.020101 0.040405 0.060918 0.081644 0.102587 0.123751 0.145141 0.166763 0.188621 0.210721 0.233068 0.255667 0.278524 0.301646 0.325038 0.348707 0.372659 0.396902 0.421442 0.446287 0.471445 0.496923 0.522730 0.548874 0.575364 0.602210 0.629421 0.657008 0.684981 0.713350 0.742127 0.771325 0.800955 0.831031 0.861566 0.892574 0.924071 0.956072 0.988593 1.021651 1.055265 1.089454 1.124238 1.159637 1.195674 1.232372 1.269757 1.307853 1.346689 1.386294 1.426700 1.467938 1.510045 1.553058 1.597015 1.641961 1.687940 1.735001 1.783196 1.832581 1.883217 1.935168 1.988505 2.043302 2.099644 2.157619 2.217325 2.278869 2.342366 2.407946 2.475749 2.545931 2.618667 2.694147 2.772589 2.854233 2.939352 3.028255 3.121295 3.218876 3.321462 3.429597 3.543914 3.665163 3.794240 3.932226 4.080442 4.240527 4.414550 4.605170 4.815891 5.051457 5.318520 5.626821 5.991465 6.437752 7.013116 7.824046 9.210340 Inf]
    f = interp1(c, x, confidence)

    return f

class Error(Exception):
    """Base class for other exceptions"""
    pass
