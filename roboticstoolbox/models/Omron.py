#!/usr/bin/env python

import numpy as np
from spatialmath.base import trotz, transl
from roboticstoolbox.robot.ETS import ETS
from roboticstoolbox.robot.ET import ET


class Omron(ETS):
    """
    A class representing the Franka Emika Panda robot arm. ETS taken from [1]
    based on https://frankaemika.github.io/docs/control_parameters.html

    :param et_list: List of elementary transforms which represent the robot
        kinematics
    :type et_list: list of etb.robot.et
    :param q_idx: List of indexes within the ets_list which correspond to
        joints
    :type q_idx: list of int
    :param name: Name of the robot
    :type name: str, optional
    :param manufacturer: Manufacturer of the robot
    :type manufacturer: str, optional
    :param base: Location of the base is the world frame
    :type base: float np.ndarray(4,4), optional
    :param tool: Offset of the flange of the robot to the end-effector
    :type tool: float np.ndarray(4,4), optional
    :param qz: The zero joint angle configuration of the robot
    :type qz: float np.ndarray(7,)
    :param qr: The ready state joint angle configuration of the robot
    :type qr: float np.ndarray(7,)

    References: [1] Kinematic Derivatives using the Elementary Transform
        Sequence, J. Haviland and P. Corke
    """
    def __init__(self):

        et_list = [
            ET.TRz(joint=1),
            ET.Ttx(joint=2)
        ]

        q_idx = [0, 1]

        super(Omron, self).__init__(
            et_list,
            q_idx,
            name='Omron',
            manufacturer='Omron')

        self._qz = np.array([0, 0])
        self._qr = np.array([0, 0])

    @property
    def qz(self):
        return self._qz

    @property
    def qr(self):
        return self._qr
