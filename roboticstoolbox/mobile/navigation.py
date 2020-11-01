"""
Python Navigation Abstract Class
@Author: Kristian Gibson
TODO: Comments + Sphynx Docs Structured Text
TODO: Bug-fix, testing

Not ready for use yet.
"""
from numpy import disp
from scipy import integrate
from spatialmath.base.transforms2d import *
from spatialmath.base.vectors import *
from spatialmath.pose2d import SE2
from scipy.ndimage import *
from matplotlib import cm
from abc import ABCMeta, abstractmethod


class Navigation(object):
    __metaclass__ = ABCMeta

    def __init__(self, occ_grid=None, goal=np.array([]), inflate=0,
                 private=False, reset=False, verbose=None, seed=np.array([]),
                 transform=SE2()):
        self._occ_grid_nav = None
        self._start = None
        self._verbose = verbose
        self._seed = seed
        self._spin_count = None
        self._rand_stream = None
        self._seed_0 = None
        self._w2g = None
        self._inflate = inflate
        self._private = private
        self._reset = reset
        self._transform = transform
        self._goal = None

        # This code is more complicated in the matlab version... but this is fine
        if occ_grid is not None:
            # this is the inverse of the matlab code
            if type(occ_grid) is dict:
                self._occ_grid = occ_grid["map"]
                self._w2g = self._T  # TODO: What in god's name is T
            else:
                self._occ_grid = occ_grid
                self.w2g = SE2(0, 0, 0)

        if inflate > 0:
            self._occ_grid_nav = binary_dilation(self._occ_grid, disk_struct(self._inflate))
        else:
            self._occ_grid_nav = occ_grid

        if goal != np.array([]):
            self._goal = np.transpose(goal)

        # Simplification of matlab code
        rs = np.random.RandomState()
        if seed != np.array([]):
            rs = np.random.RandomState(seed)

        self._seed_0 = rs.get_state()
        self._rand_stream = rs
        self._w2g = transform
        self._spin_count = 0

    @property
    def occ_grid(self):
        return self._occ_grid

    @property
    def occ_grid_nav(self):
        return self._occ_grid_nav

    @property
    def goal(self):
        return self._goal

    @property
    def start(self):
        return self._start

    @property
    def verbose(self):
        return self._verbose

    @property
    def seed(self):
        return self._seed

    @property
    def spin_count(self):
        return self._spin_count

    @property
    def rand_stream(self):
        return self._rand_stream

    @property
    def seed_0(self):
        return self._seed_0

    @property
    def w2g(self):
        return self._w2g

    @property
    def inflate(self):
        return self._inflate

    @property
    def private(self):
        return self._private

    @property
    def reset(self):
        return self._reset

    @property
    def transform(self):
        return self._transform

    # Define abstract classes to be implemented later
    @abstractmethod
    def plan(self):
        pass

    @abstractmethod
    def next(self):
        pass

    def query(self, start, animate=False):
        self.check_query(start)

        if animate:
            self.plot()

        robot = self._start
        path = self._start[:]

        while True:
            if animate:
                self.plot(robot[0], robot[1], 'g.', 12)
                plt.show()

            robot = self._next(robot)

            if robot is None:
                path = np.array([path, self._goal[:]])
                break
            else:
                path = np.array([path, robot[:]])

        # TODO: Fix?
        if path != self._start[:]:
            return np.transpose(path)

    def plot(self, p=None, distance=None, color_map=None, beta=None, inflated=None):
        self.plot_bg(p, distance, color_map, beta, inflated)
        self.plot_fg(p, distance, color_map, beta, inflated)

    def plot_bg(self, p=None, distance=np.array([]), color_map=cm.get_cmap('bone'), beta=0.2,
                inflated=False, path_marker=None, start_marker=None, goal_marker=None):
        occ_grid = None
        image = None
        if inflated:
            occ_grid = self._occ_grid_nav
        else:
            occ_grid = self._occ_grid

        if distance == np.array([]) or np.all(np.all(not np.isfinite(distance))):
            c_map = np.array([(1, 1, 1), (1, 0, 0)])
            plt.figimage(occ_grid+1, cmap=c_map)  # Todo: check that this looks fine..?
        else:
            d = distance(np.isfinite(distance))
            d = d + 2
            max_dist = np.max(d[:])
            c_map = np.array([(1, 0, 0), (color_map(np.ceil(max_dist)))])
            distance = distance + 2
            for i in range(0, len(distance)):
                if distance[i] is None:
                    distance[i] = distance + 2
            distance[occ_grid > 0] = 1

            plt.figimage(distance, cmap=c_map)
            # TODO: renderer, fix?
            scalar_mappable_c_map = cm.ScalarMappable(cmap=c_map)
            plt.colorbar(scalar_mappable_c_map)

        plt.yticks(label="Ydir")
        plt.xlabel('x')
        plt.ylabel('y')
        plt.grid(True)

    def plot_fg(self, p=None, path_marker=None, start_marker=None, goal_marker=None, goal=None):
        args_check = path_marker

        path_marker = {'marker': '.', 'markerfacecolor': 'g', 'markersize': 12 }
        start_marker = {'marker': 'o', 'markeredgecolor': 'w', 'markerfacecolor': 'b', 'markersize': 12}
        goal_marker = {'marker': 'p', 'markeredgecolor': 'w', 'markerfacecolor': 'b', 'markersize': 12}

        if p is not None and np.isnumeric(p):
            if len(p) < 2:
                Error("Expecting Nx2 or Nx3 matrix of points.")
            if len(p) == 2:
                self.plot(p[:, 0], p[:, 1], path_marker, path_marker)
            else:
                self.plot(p[:, 0], p[:, 1], p[:, 2], path_marker, path_marker)

        if len(self._goal == 2):
            if self._goal is not None:
                self.plot(self._goal[0], self._goal[1], goal_marker)
            if self._start is not None:
                self.plot(self._start[0], self._start[1], start_marker)
        else:
            if self._goal is not None:
                self.plot3(self._goal[0], self._goal[1], self._goal[2]+0.1, goal_marker)
            if self._start is not None:
                self.plot3(self._start[0], self._start[1], self._goal[2]+0.1, start_marker)

        plt.show()

    def display(self):
        disp(self.char())

    def char(self):
        s = "Navigation class"

        s = s + "\nOccupancy grid: " + np.size(self._occ_grid)
        if self._goal is not None:
            s = s + "\nGoal: " + self._goal
        return s

    def set_goal(self, goal=np.array([])):
        start = None
        if goal is None or goal is np.array([]):
            self.plot()
            disp("Select goal location")
            goal = round(plt.ginput(1))

        self._goal = goal[:]

        if self.is_occupied(self._goal):
            Error("Navigation:checkquery:badparam. Goal location inside obstacle")

    def check_query(self, start=np.array([]), goal=np.array([])):
        if start is None or goal is np.array([]):
            self.plot()
            disp("Select start location")
            start = round(plt.ginput(1))

        if goal is None or goal is np.array([]):
            self.plot()
            disp("Select goal location")
            goal = round(plt.ginput(1))

        self._start = np.transpose(start)
        self._goal = np.transpose(goal)

        assert(not self.is_occupied(self._start[0:1]),
               "Navigation:checkquery:badparam. Start location inside obstacle")
        assert(not self.is_occupied(self.goal[0:1]),
               "Navigation:checkquery:badparam. Goal location inside obstacle")

    def is_occupied(self, x=None, y=None):
        occ = None
        pis = None
        if self._occ_grid_nav is None or self._occ_grid_nav is np.array([]):
            occ = False
            return occ

        if x is not None:
            if np.size(x) == 2:
                x = np.transpose(x)
            assert(np.shape(x, 0) == 2, "RTB:Navigation:isoccupied. P must have 2 rows")
            pos = x
        else:
            assert(np.size(x) == np.size(y), "RTB:Navigation:isoccupied. X and Y must be same length")
            pos = np.array([(np.transpose(x)), np.transpose(y)])

        pos = round(self._w2g * pos)
        k = pos[0, :] > 0 & pos[0, :] <= np.shape(self._occ_grid) & pos[1,:] > 0 <= np.shape(self._occ_grid, 1)

        i = sub2ind(np.shape(self._occ_grid), pos[1, k], pos[0, k])
        occ = np.ones(1, np.size(pos, 1))  # TODO: this bit normally says 'logic' in matlab... should be fine
        occ[k] = self._occ_grid_nav[i] > 0
        return occ

    def goal_change(self):
        pass

    def navigate_init(self, start):
        pass

    def rand(self, L=None, M=None):
        r = None
        if L is None and M is None:
            r = self._rand_stream.rand()
        elif L is None:
            r = self._rand_stream.rand(M, M)
        else:
            r = self._rand_stream.rand(L, M)

        return r

    def randn(self, L=None, M=None):
        r = None
        if L is None and M is None:
            r = self._rand_stream.randn()
        elif L is None:
            r = self._rand_stream.randn(M, M)
        else:
            r = self._rand_stream.randn(L, M)

        return r

    def randi(self, RM=None, L=None, M=None):
        r = None
        if M is None and L is None:
            r = self._rand_stream.randint(1, RM)
        elif L is None:
            r = self._rand_stream.randint(1, RM, M)
        else:
            r = self._rand_stream.randint(1, RM, size=(L, M))

    def verbosity(self, v):
        self._verbose = v

    def message(self, S=None, FMT=None, ARGS=None):
        if self._verbose:
            disp(self + "\nDebug::" + ARGS + "\n")

    def spinner(self):
        spin_chars = "\|/"  # TODO: This might break?
        self._spin_count = self._spin_count + 1
        disp(spin_chars[np.mod(self._spin_count, len(spin_chars))+1])

    @staticmethod
    def show_distance(d):
        d[np.isinf(d)] = None
        ax = plt.gca()
        c_map = plt.get_cmap("Greys")
        plt.clim(0, np.max(d[:]))
        plt.figimage(d)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.show()

    # There's no waitbar in matplotlib, so just going with placeholder functions
    @staticmethod
    def progress_init(title):
        h = "Waiting"
        return h

    @staticmethod
    def progress(h, x):
        pass

    @staticmethod
    def progress_delete(h):
        plt.clf()

# Generates a structuring element like kcircle from Peter Corke's Machine Learning Toolbox, but with just a radius
# generate_binary_structure, even with iteration isn't perfect. Python CV toolboxes have equivalent functions.
def disk_struct(r):
    if r == 0:
        return np.array([False]).astype(int)
    y, x = np.ogrid[-r: r+1, -r: r+1]
    struct = np.square(x) + np.square(y) <= np.square(r)
    return struct.astype(int)

# Sourced from: https://stackoverflow.com/questions/28995146/matlab-ind2sub-equivalent-in-python/28995315#28995315
def sub2ind(array_shape, rows, cols):
    ind = rows*array_shape[1] + cols
    ind[ind < 0] = -1
    ind[ind >= array_shape[0]*array_shape[1]] = -1
    return ind

class Error(Exception):
    """Base class for other exceptions"""
    pass