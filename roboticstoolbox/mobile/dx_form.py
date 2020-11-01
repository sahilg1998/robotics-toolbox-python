"""
Python ReedShepp Planner
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
from spatialmath.base import *
from scipy.ndimage import *
from matplotlib import cm
from roboticstoolbox.mobile.navigation import Navigation


class DX_Form(Navigation):
    def __init__(self, occ_grid=None, goal=np.array([]), inflate=0,
                 private=False, reset=False, verbose=None, seed=np.array([]),
                 transform=SE2(), metric="euclidean"):
        self._metric = metric
        self._distance_map = None

    @property
    def metric(self):
        return self._metric

    @property
    def distance_map(self):
        return self._distance_map

    def char(self):
        s = "" + self
        s = s + "\nDistance metric: " + self._metric
        if self._distance_map is not None:
            s = s + ", Distance map: computed "
        else:
            s = s + ", Distance map: empty "

        return s

    def goal_change(self, goal):
        self._distance_map = np.array([])

    def plan(self, goal=None, animate=False):
        show = None
        if animate:
            show = 0.05
        else:
            show = 0

        if goal is not None and goal != np.array([]) and isvector(goal, 2):
            self.set_goal(goal)

        assert(self._goal is not None and self._goal != np.array([]), 'RTB:dx_form:plan. No goal specified '
                                                                      'here or in constructor')

        self._distance_map = self.distance_x_form(self.occ_grid_nav, self._goal, self._metric, 'show')

    # Use plot from parent class

    def next(self, robot):
        if self._distance_map is None or self._distance_map == np.array([]):
            Error("No distance map computed, you need to plan.")

        directions = np.array([
            -1, -1,
            0, -1,
            1, -1,
            -1, 0,
            0, 0,
            1, 0,
            0, 1,
            1, 1
        ])

        x = robot[0]
        y = robot[1]

        min_dist = np.inf
        min_dir = np.array([])
        for d in np.transpose(directions):
            try:
                if self._distance_map[y + d[0], x + d[1]] < min_dist:
                    min_dir = d
                    min_dist = self.distance_map[y + d[0], x + d[1]]
            except:
                Error("Unexpected error finding next min dist at d: " + d)

        x = x + min_dir[1]
        y = y + min_dir[0]

        n = None
        if np.all(np.array([[x], [y]]) == self._goal):
            n = np.array([])
        else:
            n = np.array([[x], [y]])

        return n

    def plot_3d(self, p=None, ls=None):
        fig = plt.figure()
        ax = fig.gca(projection='3d')

        surf = ax.plot_surface(self._distance_map, cmap=cm.coolwarm,
                               linewidth=0, antialiased=False)

        if p is not None:
            k = sub2ind(np.shape(self._distance_map), p[:, 1], p[:, 0])
            height = self._distance_map[k]
            ax.plot(p[:, 0], p[:, 1], height)

        plt.show()


# Sourced from: https://stackoverflow.com/questions/28995146/matlab-ind2sub-equivalent-in-python/28995315#28995315
def sub2ind(array_shape, rows, cols):
    ind = rows*array_shape[1] + cols
    ind[ind < 0] = -1
    ind[ind >= array_shape[0]*array_shape[1]] = -1
    return ind


def ind2sub(array_shape, ind):
    ind[ind < 0] = -1
    ind[ind >= array_shape[0]*array_shape[1]] = -1
    rows = (ind.astype('int') / array_shape[1])
    cols = ind % array_shape[1]
    return rows, cols


class Error(Exception):
    """Base class for other exceptions"""
    pass