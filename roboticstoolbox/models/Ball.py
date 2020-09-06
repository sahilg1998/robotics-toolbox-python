"""
@author: Peter Corke
@author: Samuel Drew
"""

from roboticstoolbox.robot.serial_link import *
from roboticstoolbox.robot.Link import RevoluteDH
from math import pi
import numpy as np


class Ball(SerialLink):

    # %MDL_BALL Create model of a ball manipulator
    # %
    # % MDL_BALL creates the workspace variable ball which describes the
    # % kinematic characteristics of a serial link manipulator with 50 joints
    # % that folds into a ball shape.
    # %
    # % MDL_BALL(N) as above but creates a manipulator with N joints.
    # %
    # % Also define the workspace vectors:
    # %   q  joint angle vector for default ball configuration
    # %
    # % Reference::
    # % - "A divide and conquer articulated-body algorithm for parallel O(log(n))
    # %   calculation of rigid body dynamics, Part 2",
    # %   Int. J. Robotics Research, 18(9), pp 876-892.
    # %
    # % Notes::
    # % - Unlike most other mdl_xxx scripts this one is actually a function that
    # %   behaves like a script and writes to the global workspace.
    # %
    # % See also mdl_coil, SerialLink.
    #
    # % MODEL: generic, ball shape, hyper redundant, 50DOF, standard_DH
    #
    #
    # % Copyright (C) 1993-2017, by Peter I. Corke
    # %
    # % This file is part of The Robotics Toolbox for MATLAB (RTB).
    # %
    # % RTB is free software: you can redistribute it and/or modify
    # % it under the terms of the GNU Lesser General Public License as published by
    # % the Free Software Foundation, either version 3 of the License, or
    # % (at your option) any later version.
    # %
    # % RTB is distributed in the hope that it will be useful,
    # % but WITHOUT ANY WARRANTY; without even the implied warranty of
    # % MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    # % GNU Lesser General Public License for more details.
    # %
    # % You should have received a copy of the GNU Leser General Public License
    # % along with RTB.  If not, see <http://www.gnu.org/licenses/>.
    # %
    # % http://www.petercorke.com
    def __init__(self, N=None):

        links = []
        self._qz = []
        if not N:
            N = 10
        self.N = N

        for i in range(self.N):
            links.append(Link(a=0.1, alpha=pi/2))
            self._qz.append(self.fract(i+1))

        # and build a serial link manipulator
        super(Ball, self).__init__(links, name='ball')

    @property
    def qz(self):
        return self._qz

    def fract(self, i):
        theta1 = 1
        theta2 = -2/3

        out = i % 3
        if out < 1:
            f = self.fract(i/3)
        elif out < 2:
            f = theta1
        else:
            f = theta2
        return f
