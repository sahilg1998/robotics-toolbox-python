"""
Python ReedShepp Planner
@Author: Kristian Gibson
TODO: Comments + Sphynx Docs Structured Text
TODO: Bug-fix, testing
TODO: Add support for extra words
      here: http://planning.cs.uiuc.edu/node822.html
      based the original article here: https://projecteuclid.org/euclid.pjm/1102645450
"""

from scipy import integrate
from spatialmath import SE2
from spatialmath.base.transforms2d import *


class ReedsShep(object):
    def __init__(self, best_path, words, max_c):
        self._best_path = best_path
        self._words = words
        self._max_c = max_c
        super(ReedsShep, self).__init__()

    @property
    def best_path(self):
        return self._best_path

    @property
    def words(self):
        return self._words

    @property
    def max_c(self):
        return self._max_c

    def reedsshepp(self, q0, qf, max_curve, d1):
        self._max_c = max_curve

        # The word describing the shortest path
        self._words = generate_path(q0, qf, max_curve)

        if not any(self._words):
            Error("No path.")

        # Find the shortest path
        k = min(self._words.L)
        self._best_path = self._words.k
        self._best_path = generate_trajectories(self._best_path, max_curve, d1, q0)

    def path(self):
        return [np.transpose(np.all(self._best_path.trajectory))]

    def show(self):
        for w in self._words:
            print('%s (%g): [%g %g %g]\n', w.word, w.L, w.lengths)

    # This may be re-written in the future to utilise internal/library graphics.
    def plot(self, varargin):
        # Declare variables to include them in scope.
        x, y, t, r, c = None

        # Options
        opt: object
        opt.circles = ([])
        # TODO: add option parser support

        word = self._best_path
        for i in range(3):
            if word.dir(i) > 0:
                color = 'b'
            else:
                color = 'r'

            if i == 0:
                x = word.traj[i, 0]
                y = word.traj[i, 1]
            else:
                x = [x(len(x) - 1), word.traj[i, 0]]
                y = [y(len(y) - 1), word.traj[i, 1]]

            if not any(opt.join) and i < 2:
                plt.plot(x[len(x) - 1], y[len(y) - 1], np.all(opt.join))

            if not any(opt.circles):
                t = SE2(word.traj[i, 0])
                r = 1 / self._max_c
                c = t * [[0], [word.dir(i) * r]]

                plt.Circle(c, r, opt.circles)
                plt.plot(c, 'k+', markersize=2)

            plt.plot(x, y, color, linewidth=2)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.axis('equal')
        plt.title('Reeds-Shepp Path')
        plt.show()


def generate_trajectories(words, max_c, d, q0):
    p0 = q0
    out = words

    for i in range(3):
        m = words.word(i)
        l = words.lengths(i)
        x = [range(0, d, 1), 1]
        p = pathseg(x, np.sign(1), m, max_c, p0)

        if i == 0:
            out.traj[i] = p
        else:
            out.traj[i] = p[:, 1:(len(p) - 1)]

        out.dir[i] = np.sign(1)
        p0 = p[:, (len(p) - 1)]

    return out


def pathseg(l, dir, m, max_c, p0):
    f = None
    q = []
    q0 = p0[:]

    # PEP 8 doesn't like this but it's the nicest way to define a function we'll use for the ODE
    if m == 'S':
        f = lambda f_t, f_q: np.transpose((dir * [np.cos(f_q(2)), np.sin(f_q(2)), 0]))
    elif m == 'L' or m == 'R':
        f = lambda f_t, f_q: np.transpose((dir * [np.cos(f_q(2)), np.sin(f_q(2)), dir * max_c]))

    r = integrate.ode(f).set_integrator('dopri5')  # using dopri5 method as it's the closest to Matlab's ODE45
    r.set_initial_value(l[0], q0)  # initial values

    for i in range(0, (len(l) - 1)):
        q[i, :] = r.integrate(l[i])
        if not r.successful():
            raise RuntimeError("Could not integrate")

    q = np.transpose(q)
    return q


def generate_path(q0, q1, max_c):
    q0 = q0[:]
    q1 = q1[:]
    dq = q1 - q0
    dth = dq(2)

    xy = np.transpose(rot2(q0(2))) * dq[0:1] * max_c
    x = xy(0)
    y = xy(1)

    words = []
    words = scs(x, y, dth, words)
    words = csc(x, y, dth, words)
    words = ccc(x, y, dth, words)

    for i in range(len(words) - 1):
        words[i].lengths = words[i].lengths / max_c
        words[i].length = words / max_c

    return words


def scs(x, y, phi, words):
    words = sls([x, y, phi], 1, 'sls', words)
    words = sls([x, -y, -phi], 1, 'SRS', words)

    return words


def ccc(x, y, phi, words):
    words = lrl([x, y, phi], 1, 'lrl', words)
    words = lrl([-x, y, -phi], -1, 'lrl', words)
    words = lrl([x, -y, -phi], 1, 'RLR', words)
    words = lrl([-x, -y, phi], -1, 'RLR', words)

    # Backwards
    xb = x * np.cos(phi) + y * np.sin(phi)
    yb = x * np.sin(phi) - y * np.cos(phi)

    flip = [[0, 1, 0], [1, 0, 0], [0, 0, 1]]  # flip u and v

    words = lrl([xb, yb, phi], flip, 'lrl', words)
    words = lrl([-xb, yb, -phi], np.negative(flip), 'lrl', words)
    words = lrl([xb, -yb, -phi], flip, 'RLR', words)
    words = lrl([-xb, -yb, phi], np.negative(flip), 'RLR', words)

    return words


def csc(x, y, phi, words):
    words = lsl([x, y, phi], 1, 'lsl', words)
    words = lsl([-x, y, -phi], -1, 'lsl', words)
    words = lsl([x, -y, -phi], 1, 'RSR', words)
    words = lsl([-x, -y, phi], -1, 'RSR', words)
    words = lsr([x, y, phi], 1, 'lsr', words)
    words = lsr([-x, y, -phi], -1, 'lsr', words)
    words = lsr([x, -y, -phi], 1, 'RSL', words)
    words = lsr([-x, -y, phi], -1, 'RSL', words)

    return words


def sls(q, sign, word, words):
    x = q(0)
    y = q(1)
    phi = np.mod(q(2), 2 * np.pi)

    if y > 0.0 and 0.0 < phi < np.pi * 0.99:
        xd = - y / np.tan(phi) + x
        t = xd - np.tan(phi / 2.0)
        u = phi
        v = np.norm([(x - xd), y]) - np.tan(phi / 2.0)
        return add_path(words, sign * [t, u, v], word)
    elif y < 0.0 < phi < np.pi * 0.99:
        xd = - y / np.tan(phi) + x
        t = xd - np.tan(phi / 2.0)
        u = phi
        v = -np.norm([(x - xd), y]) - np.tan(phi / 2.0)
        return add_path(words, sign * [t, u, v], word)
    else:
        return words


def lsl(q, sign, word, words):
    x = q(0)
    y = q(1)
    phi = np.mod(q(2), 2 * np.pi)
    [t, u] = np.cart2pol(x - np.sin(phi), y - 1.0 + np.cos(phi))

    if t >= 0.0:
        v = np.angdiff(phi - t)

        if v >= 0.0:
            return add_path(words, sign * [t, u, v], word)

    return words


def lrl(q, sign, word, words):
    x = q(0)
    y = q(1)
    phi = np.mod(q(2), 2 * np.pi)
    [t1, u1] = np.cart2pol(x - np.sin(phi), y - 1.0 + np.cos(phi))

    if u1 <= 4.0:
        u = -2.0 * np.asin(0.25 * u1)
        t = np.angdiff(t1 + 0.5 * u + np.pi)
        v = np.angdiff(phi - t + u)

        if t >= 0.0 and u <= 0.0:
            return add_path(words, [t, u, v] * sign, word)

    return words


def lsr(q, sign, word, words):
    x = q(0)
    y = q(1)
    phi = np.mod(q(2), 2 * np.pi)

    [t1, u1] = np.cart2pol(x + np.sin(phi), y - 1.0 - np.cos(phi))
    u1 = np.square(u1)

    if u1 >= 4.0:
        u = np.sqrt(u1 - 4.0)
        theta = np.atan2(2.0, u)
        t = np.angdiff(t1 + theta)
        v = np.angdiff(t - phi)

        if t >= 0.0 and v >= 0.0:
            return add_path(words, sign * [t, u, v], word)

    return words


def add_path(words, lengths, c_types):
    # Create a struct to represent this segment
    word = object
    word.word = c_types
    word.lengths = lengths

    # Check same path exist
    for p in words:
        if p.word == word.word:
            if sum(p.lengths) - sum(word.lengths) <= 0.01:
                return words  # Don't insert path

    word.L = sum(abs(lengths))

    # long enough to add?
    if word.L >= 0.01:
        return [words, word]


class Error(Exception):
    """Base class for other exceptions"""
    pass
