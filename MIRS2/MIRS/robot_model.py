import math
import numpy as np

UNIT_FACTOR = 1  # in m (*1000 for unit in mm)


class RobotModel:
    def __init__(self, n=3, unit=UNIT_FACTOR):
        # Number of DOFs
        self.n = n
        self.unit = unit

        # Length along two Z axces LINK_
        # Length between two z axces LINK_T
        # LINK_0_LENGTH = 0
        # LINK_1_LENGTH = 100
        # LINK_T_2_LENGTH = 70
        # LINK_2_LENGTH = 250
        # LINK_3_LENGTH = 250
        # LINK_T_4_LENGTH = 70
        # LINK_T_5_LENGTH = 70
        # LINK_5_LENGTH = 50
        # LINK_6_LENGTH = 60

        # Link parameters (in m)
        # Link length

        l0 = 0.2*unit
        l1 = 0.3*unit
        l2 = 0.25*unit

        self.H_MAX = l0+l1+l2+(.025*unit)

        self.link_lengths = [l0, l1, l2]

        # Link radii
        # L0
        d0 = 0.06*unit
        r0 = 0.03*unit

        # L1
        d1 = 0.05*unit
        r1 = 0.025*unit

        # L2
        d2 = 0.04*unit
        r2 = 0.02*unit

        # L2
        # Link density (in kg/m^3)
        rho = 2710/(unit**3)

        # nx2x4
        self.links = [
            [[0, 0, 0, 1], [0, 0, l0, 1]],
            [[0, 0, 0, 1], [l1, 0, 0, 1]],
            [[0, 0, 0, 1], [l2, 0, 0, 1]],
        ]
        self.joint_limits = [(0, 360), (-45, 225), (-45, 225)]
        # DH parameters
        # nx4 matrix of DH parameters
        self.DH_PARAMS = [[0, math.pi/2, l0, 0],
                          [l1, 0, 0, 0],
                          [l2, 0, 0, 0]]

        # Link masses (in Kg)
        m0 = rho*(0.25*math.pi*d0**2*l0)
        m1 = rho*(0.25*math.pi*d1**2*l1)
        m2 = rho*(0.25*math.pi*d2**2*l2)

        self.link_masses = [m0, m1, m2]

        # Link Inertias (in Kgm^2)
        I0xx = 0.25*m0*r0**2 + 0.333*m0*l0**2  # At end
        I0yy = 0.25*m0*r0**2 + 0.33*m0*l0**2  # At end
        I0zz = 0.5*m0*r0**2  # At axis
        I0xy = 0
        I0xz = 0
        I0yz = 0

        I1xx = 0.5*m1*r1**2  # At axis
        I1yy = 0.25*m1*r1**2 + 0.333*m1*l1**2  # At end
        I1zz = 0.25*m1*r1**2 + 0.333*m1*l1**2  # At end
        I1xy = 0
        I1xz = 0
        I1yz = 0

        I2xx = 0.5*m2*r2**2  # At axis
        I2yy = 0.25*m2*r2**2 + 0.333*m2*l2**2  # At end
        I2zz = 0.25*m2*r2**2 + 0.333*m2*l2**2  # At end
        I2xy = 0
        I2xz = 0
        I2yz = 0
        self.r_bar = np.array([[0, 0, l0/2, 1],
                               [l1/2, 0, 0, 1],
                               [l2/2, 0, 0, 1]]).T
        self.link_inertias = [[I0xx, I0yy, I0zz, I0xy, I0xz, I0yz],
                              [I1xx, I1yy, I1zz, I1xy, I1xz, I1yz],
                              [I2xx, I2yy, I2zz, I2xy, I2xz, I2yz]]
