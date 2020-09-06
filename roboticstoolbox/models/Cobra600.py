"""
@author: Peter Corke
@author: Samuel Drew
"""

from roboticstoolbox.robot.serial_link import *
from roboticstoolbox.robot.Link import RevoluteDH
from math import pi
import numpy as np


class Cobra600(SerialLink):

    # %MDL_COBRA600 Create model of Adept Cobra 600 manipulator
    # %
    # % MDL_COBRA600 is a script that creates the workspace variable c600 which
    # % describes the kinematic characteristics of the 4-axis Adept Cobra 600
    # % SCARA manipulator using standard DH conventions.
    # %
    # % Also define the workspace vectors:
    # %   qz         zero joint angle configuration
    # %
    # % Notes::
    # % - SI units are used.
    # %
    # % See also SerialRevolute, mdl_puma560akb, mdl_stanford.
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
    #
    # % MODEL: Adept, Cobra600, 4DOF, standard_DH
    #
    # % hardstop limits included
    def __init__(self):
        deg = pi/180

        L = [RevoluteDH(d=0.387, a=0.325, qlim=[-50*deg, 50*deg]),
             RevoluteDH(a=0.275, alpha=pi, qlim=[-88*deg, 88*deg]),
             PrismaticDH(qlim=[0, 0.210]),
             RevoluteDH()]

        super(Cobra600, self).__init__(L, name='Cobra600', manufacturer='Adept')

        self._qz = [0, 0, 0, 0]

    @property
    def qz(self):
        return self._qz
