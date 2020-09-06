"""
@author: Peter Corke
@author: Samuel Drew
"""

from roboticstoolbox.robot.serial_link import *
from roboticstoolbox.robot.Link import RevoluteDH
from math import pi
import numpy as np


    # %MDL_FANUC10L  Create kinematic model of Fanuc AM120iB/10L robot
    # %
    # % MDL_FANUC10L is a script that creates the workspace variable R which
    # % describes the kinematic characteristics of a Fanuc AM120iB/10L robot
    # % using standard DH conventions.
    # %
    # % Also defines the workspace vector:
    # %   q0   mastering position.
    # %
    # % Notes::
    # % - SI units of metres are used.
    # %
    # % Author::
    # %  Wynand Swart,
    # %  Mega Robots CC, P/O Box 8412, Pretoria, 0001, South Africa,
    # %  wynand.swart@gmail.com
    # %
    # % See also mdl_irb140, mdl_m16, mdl_motomanHP6, mdl_puma560, SerialLink.
    #
    # % MODEL: Fanuc, AM120iB/10L, 6DOF, standard_DH
    #
    # % Copyright (C) 1993-2011, by Peter I. Corke
    # %
    # % This file is part of The Robotics Toolbox for Matlab (RTB).
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
    #
    # %Cell: 073-1555-430
    # %30 Sep 2007
# %Fanuc AM120iB/10L robot
L1 = Link(a=0.15, alpha=-pi/2)
L2 = Link(a=0.77, alpha=pi)
L3 = Link(a=0.1, alpha=-pi/2)
L4 = Link(d=-0.96, alpha=pi/2)
L5 = Link(a=0, alpha=-pi/2)
L6 = Link(d=-0.1)
L = [L1, L2, L3, L4, L5, L6]

q0 = [0, -pi/2, 0, 0, 0, 0]

R = SerialLink(L, 'name', 'Fanuc AM120iB/10L')