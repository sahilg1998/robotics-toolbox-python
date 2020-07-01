"""
Link object v2
Python implementation by Samuel Drew
"""

from numpy import *
from spatialmath.pose3d import *


class Link(list):
    """
    Create a Link object
    """

    def __init__(self, *argv,
                 alpha=0,
                 a=0,
                 theta=0,
                 d=0,
                 jointtype='R',
                 mdh=0,
                 offset=0,
                 qlim=None,
                 flip=False,
                 Tc=None,
                 Jm=0,
                 I=None,
                 m=0,
                 r=None,
                 sym=False):

        if r is None:
            self.r = [0, 0, 0]
        if Tc is None:
            self.Tc = [0, 0]
        if qlim is None:
            self.qlim = [-pi / 2, pi / 2]
        if I is None:
            self.I = zeros([3, 3])

        if not argv:
            print("Creating Link class object")
            """
            Create a Link object using named arguments
            This call signature is needed to support arrays of Links
            """
            assert d == 0 or theta == 0, "Bad argument, cannot specify both d and theta"
            assert jointtype == 'R' or jointtype == 'P', "Bad argument, jointtype must be 'R' or 'P'"

            if theta != 0:
                # constant value of theta means it must be prismatic
                jointtype = 'P'

            if jointtype == 'R':
                self.jointtype = 'R'
                assert theta == 0, "Bad argument, cannot specify 'theta' for revolute joint"
            elif jointtype == 'P':
                self.jointtype = 'P'
                assert d == 0, "Bad argument, cannot specify 'd' for prismatic joint"

            self.alpha = alpha
            self.a = a
            self.theta = theta
            self.d = d
            self.offset = offset
            self.qlim = qlim
            self.flip = flip
            self.Tc = Tc
            self.Jm = Jm
            self.I = I
            self.m = m
            self.r = r
            self.sym = sym
            self.mdh = mdh

        elif len(argv) == 1 and isinstance(argv, Link):
            # Clone the passed Link object
            self = argv[0]

    def __repr__(self):

        if not self.mdh:
            conv = 'std'
        else:
            conv = 'mod'

        if self.jointtype == 'R':
            return "Revolute("+conv+") joint with attributes: d = "+\
                   str(self.d)+", a = "+str(self.a)+", alpha = "+str(self.alpha)
        elif self.jointtype == 'P':
            return "Prismatic("+conv+") joint with attributes: theta = "+\
                   str(self.theta)+", a = "+str(self.a)+", alpha = "+str(self.alpha)
        else:
            return "jointtype unspecified"

    def type(self):
        """
        Link.type Joint type

        c = L.type() is a character'R' or 'P' depending on whether
        joint is revolute or prismatic respectively.
        TODO If L is a list vector of Link objects return an array of characters in joint order.
        """
        return self.jointtype

    def isrevolute(self):
        """
        Link.isrevolute() Test if joint is revolute
        returns True id joint is revolute
        """
        return self.jointtype == 'R'

    def isprismatic(self):
        """
        Link.isprismatic() Test if joint is prismatic
        returns True id joint is prismatic
        """
        return self.jointtype == 'P'

    def A(self, q):
        """
        Link.A Link transform matrix

        T = L.A(q) is an SE3 object representing the transformation between link
        frames when the link variable q which is either the Denavit-Hartenberg
        parameter theta (revolute) or d (prismatic).  For:
         - standard DH parameters, this is from the previous frame to the current.
         - modified DH parameters, this is from the current frame to the next.

        Notes::
        - For a revolute joint the THETA parameter of the link is ignored, and Q used instead.
        - For a prismatic joint the D parameter of the link is ignored, and Q used instead.
        - The link offset parameter is added to Q before computation of the transformation matrix.
        """
        sa = sin(self.alpha)
        ca = cos(self.alpha)
        if self.flip:
            q = -q + self.offset
        else:
            q = q + self.offset
        if self.isrevolute():
            # revolute
            st = sin(q)
            ct = cos(q)
            d = self.d
        else:
            # prismatic
            st = sin(self.theta)
            ct = cos(self.theta)
            d = q

        if not self.mdh:
            # standard DH

            T = array([[ct, -st*ca, st*sa, self.a*ct],
                       [st, ct*ca, -ct*sa, self.a*st],
                       [0, sa, ca, d],
                       [0, 0, 0, 1]])
        else:
            # modified DH

            T = array([[ct, -st, 0, self.a],
                       [st*ca, ct*ca, -sa, -sa*d],
                       [st*sa, ct*sa, ca, ca*d],
                       [0, 0, 0, 1]])

        return SE3(T)

