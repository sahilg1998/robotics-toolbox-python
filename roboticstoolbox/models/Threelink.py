"""
Defines the object 'tl' in the current workspace

Also define the vector qz = [0 0 0] which corresponds to the zero joint
angle configuration.

@author: Luis Fernando Lara Tobar and Peter Corke

Edited June 2020 by Samuel Drew
"""

from roboticstoolbox.robot.serial_link import *


class Threelink(SerialLink):

    def __init__(self):

        L = [Link(a=1, jointtype='R'),
             Link(a=1, jointtype='R'),
             Link(a=1, jointtype='R')]

        self._qz = [pi/4, 0.1, 0.1]

        super(Threelink, self).__init__(L, name='Simple three link')

    @property
    def qz(self):
        return self._qz
