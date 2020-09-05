import unittest
import numpy.testing as nt
from roboticstoolbox.robot.Link import *


class TestLink(unittest.TestCase):

    def test_create_default_joint(self):
        L1 = Link()

        self.assertIsInstance(L1, Link)

    def test_create_dval_rev_joint(self):
        L1 = Link(d=1)

        self.assertIs(L1.jointtype, 'R')

    def test_create_thetaval_pris_joint(self):
        L1 = Link(theta=1)

        self.assertIs(L1.jointtype, 'P')

    def test_create_d_theta_valueError(self):

        self.assertRaises(ValueError, Link, theta=1, d=1)

    def test_create_d_theta_R_rotational(self):
        L1 = Link(theta=1, d=1, jointtype='R')

        self.assertIs(L1.jointtype, 'R')

    def test_create_d_theta_P_prismatic(self):
        L1 = Link(theta=1, d=1, jointtype='P')

        self.assertIs(L1.jointtype, 'P')

    def test_isrevolute_True(self):
        L1 = RevoluteDH()

        self.assertTrue(L1.isrevolute())

    def test_isrevolute_False(self):
        L1 = PrismaticDH()

        self.assertFalse(L1.isrevolute())

    def test_isprismatic_True(self):
        L1 = PrismaticDH()

        self.assertTrue(L1.isprismatic())

    def test_isprismatic_False(self):
        L1 = RevoluteDH()

        self.assertFalse(L1.isprismatic())

    def test_zero_angle_A(self):
        L1 = RevoluteDH()
        B = array([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])

        nt.assert_array_almost_equal(L1.A(0).A, B)


if __name__ == '__main__':
    unittest.main()
