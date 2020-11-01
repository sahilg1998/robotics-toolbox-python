"""
Python Bug Planner
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
from spatialmath.base.animate import *
from scipy.ndimage import *
from matplotlib import cm
from roboticstoolbox.mobile.navigation import Navigation


class Bug2(Navigation):
    def __init__(self, occ_grid=None, goal=np.array([]), inflate=0,
                 private=False, reset=False, verbose=None, seed=np.array([]),
                 transform=SE2()):
        self._h = np.array([])
        self._j = 1
        self._step = 1
        self._m_line = None
        self._edge = None
        self._k = None

    @property
    def h(self):
        return self._h

    @property
    def j(self):
        return self._j

    @property
    def step(self):
        return self._step

    @property
    def m_line(self):
        return self._m_line

    @property
    def edge(self):
        return self._edge

    @property
    def k(self):
        return self._k

    def query(self, start=None, goal=None, animate=False, movie=np.array([]), current=False):
        anim = None
        if movie is not None and movie != np.array([]):
            anim = Animate(movie)
            animate = True

        self._start = np.array([])
        self._goal = np.array([])
        self.check_query(start, goal)

        self._m_line = np.hom_line(self._start[0], self._start[1],
                                   self._goal[1], self._goal[1])
        self._m_line = self._m_line / np.linalg.norm

        if animate:
            self.plot()
            self.plot_m_line()

        robot = self._start[:]
        self.step = 1
        path = self._start[:]
        h = None
        while True:
            if animate:
                self.plot(robot[0], robot[1])
                if current:
                    h = self.plot(robot[0], robot[1])
                plt.draw()
                if movie is not None and movie != np.array([]):
                    anim.plot(h)
                if current:
                    self.delete(h)

            robot = self.next(robot)
            if np.empty(robot) or robot is None or robot == np.array([]):
                break
            else:
                path = np.array([path, robot[:]])

        if movie is not None and movie != np.array([]):
            anim.done()

        if path != self._start[:]:
            return np.transpose(path)

    def plot_m_line(self, ls=None):
        if ls is None:
            ls = 'k--'

        x_min, x_max, y_min, y_max = plt.axis()
        if self._m_line == 0:
            plt.plot(np.array([self._start[0, self._start[0]]]),
                     np.array([y_min, y_max]), 'k--')
        else:
            x = np.transpose(np.array([x_min, x_max]))

            y_el = np.array([x, (np.array([[1], [1]]) *
                                 np.array([[self._m_line[0]], [self._m_line[2]]]))
                           ])
            y = -y_el / self._m_line[1]
            plt.plot(x, y, ls)

    def next(self, robot):
            n = np.array([])
            robot = robot[:]
            dx = None
            dy = None
            l = None
            y = None

            if self._step == 1:
                if col_norm(self._goal - robot) == 0:
                    return n
                d = self._goal-robot
                if abs(d[0]) > np.abs(d[1]):
                    dx = np.sign(d[0])
                    l = self._mline
                    y = -((robot[0] + dx) * l[0] + l[2]) / l[1]
                    dy = round(y - robot[1])
                else:
                    dy = np.sign(d[1])
                    l = self._mline
                    x = -((robot(2) + dy) * l[1] + l[2]) / l[0]
                    dx = round(x - robot[0])

                if self.is_occupied(robot + np.array[dx, dy]):
                    self.message(n + "obstacle!")
                    self.h[self._j,:] = robot
                    self._step = 2
                    self._edge = edgelist(self._occgridnav == 0, robot)
                    self._k = 2
                else:
                    n = robot + np.array([dx, dy])

            if self._step == 2:
                if col_norm(self._goal-robot) == 0:
                    return n

                if self._k <= len(self._edge[0]):
                    n = self._edge[:, self._k]
                else:
                    Error('RTB:bug2:noplan, robot is trapped')
                    return n

                if np.abs(np.array([np.transpose(robot), 1]) * np.transpose(self._mline)) <= 0.5:
                    self.message("" +n + " moving along the M-line")
                    if col_norm(robot - self._goal) < np.transpose(col_norm(self._h[self._j, :]) - self._goal):
                        self._j = self._j + 1
                        self._step = 1
                        return n
                self._message("" + n + " keep moving around obstacle")
                self._k = self._k + 1
        
    def plan(self):
        Error('RTB:Bug2:badcall', 'This class has no plan method')

# Ported from Peter Corke's edgelist function found:
# https://github.com/petercorke/toolbox-common-matlab/blob/master/edgelist.m
def edgelist(im, p=None, direction=None):
    if direction is None:
        direction = 0

    if direction == 0:
        neighbours = np.arange(start=0, stop=7, step=1)
    else:
        neighbours = np.arange(start=7, stop=0, step=-1)

    p = p[:]
    pix0 = None
    try:
        pix0 = im(p[1], p[0])
    except:
        Error('TBCOMMON:edgelist, specified coordinate is not within image')

    p0 = np.array([])
    q = adjacent_point(im, p, pix0)

    assert(q is None or q is np.array([]), 'TBCOMMON:edgelist', 'no neighbour outside the blob')

    d = None
    e = p
    dir = np.array([])
    dirs = np.transpose(np.array([
        -1, 0,
        -1, 1,
        0, 1,
        1, 1,
        1, 0,
        1, -1,
        0, -1,
        -1, -1,
    ]))

    while True:
        dq = q - p
        for kq in range(0, 7):
            if np.all(dq == dirs[:, kq]):
                break
        for j in neighbours:
            k = j + kq
            if k > 7:
                k = k - 7
            dir = np.array([dir, k])
            nk = p + dirs[:,k]

            try:
                if im[nk[1], nk[0]] == pix0:
                    p = nk
                    break
            except:
                Error("Something went wrong calculating edgelist")

            q = nk

        if np.empty(p0) or p0 == np.array([]):
                p0 = p
        else:
            if np.all(p == p0):
                break
        e = np.array([e, p])
        d = dir

    return e, d

# Ported from Peter Corke's adjacent_point function found:
# https://github.com/petercorke/toolbox-common-matlab/blob/master/edgelist.m
def adjacent_point(im, seed, pix0):
    dirs = np.array([
        1, 0,
        0, 1,
        1, 0,
        0, -1,
        -1, 1,
        -1, -1,
        1, -1,
        1, 1])
    p = None

    for d in np.transpose(dirs):
        p = seed[:] + d
        try:
            if im(p[1], p[0]) != pix0:
                return p
        except:
            return p

    p = []
    return p


# Implementation of Peter Corke's matlab homline function from:
# https://github.com/petercorke/spatialmath-matlab/blob/master/homline.m
def hom_line(x1, y1, x2, y2):
    line = np.cross(np.array([x1, y1, 1]), np.array([x2, y2, 1]))

    # normalize so that the result of x*l' is the pixel distance
    # from the line
    line = line / np.linalg.norm(line[0:2])
    return line

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

def col_norm(x):
    y = np.array([])
    if x.ndim > 1:
        x = np.column_stack(x)
        for vector in x:
            y = np.append(y, np.linalg.norm(vector))
    else:
        y = np.linalg.norm(x)
    return y

class Error(Exception):
    """Base class for other exceptions"""
    pass