from urdf_parser_py.urdf import URDF
import numpy as np
import math

UNIT_FACTOR = 1  # in m (*1000 for unit in mm)

class RobotModel:
    def __init__(self, n=3, unit=UNIT_FACTOR,TEST_MODEL=False):
        # Number of DOFs
        self.n = n
        self.unit = unit
        self.DH_PARAMS = [None]*n
        self.link_inertias = [None]*n
        self.r_bar = [None]*n
        self.joint_names=[]
        self.link_names=[]
        self.joint_limits = [(0, 360),(0, 360),(0, 360),(0, 360),(0, 360),(0, 360)]

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


        if TEST_MODEL is True:
            self.joint_names=[
                "J1",
                "J2",
                "J3",
                "J4",
                "J5",
                "J6",
                              ]
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

    def set_DH_params(self,DH):
        self.DH_PARAMS = DH
        
    def set_link_inertia_mat(self,inertia_mat):
        self.link_inertias = inertia_mat

class URDFConverter:
    def __init__(self,urdf_file_path=None,TEST_MODEL=False):
        self.TEST_MODEL = TEST_MODEL
        if(urdf_file_path):
            urdf = URDF()
            self.robot = urdf.from_xml_file(urdf_file_path)

    def get_robot_model(self):
        robot_model = RobotModel()

        if not self.TEST_MODEL:
            self.extract_dh_parameters(robot_model)
            robot_model.n=self.n
            self.extract_inertia_matrix()
        return robot_model

    """
    Extract the inertia matrix of a robot from a URDF file.
    
    Args:
        urdf_file: Path to the URDF file
        
    Returns:
        inertia_matrix: 3x3 inertia matrix for each link of the robot
    """
    def extract_inertia_matrix(self,robot_model=None):
        inertia_matrix = {}

        for link in self.robot.links:
            if robot_model:
                    robot_model.link_names.append(link.name)
            inertial = link.inertial
            if inertial is not None:
                inertia = inertial.inertia
                inertia_matrix[link.name] = np.array([
                    [inertia.ixx, inertia.ixy, inertia.ixz],
                    [inertia.ixy, inertia.iyy, inertia.iyz],
                    [inertia.ixz, inertia.iyz, inertia.izz]
                ])
        if robot_model:
            robot_model.set_link_inertia_mat(inertia_matrix)
        else:
            return inertia_matrix
    
    """
    Extract the Denavit-Hartenberg (DH) parameters from a URDF file.
    
    Args:
        urdf_file: Path to the URDF file
        
    Returns:
        dh_parameters: List of DH parameters [a, alpha, d, theta] for each joint
    """
    def extract_dh_parameters(self,robot_model=None):
        dh_parameters = []

        self.n = 0
        for joint in self.robot.joints:
            if joint.type != 'fixed':
                self.n+=1
                if robot_model:
                    robot_model.joint_names.append(joint.name)
                origin = joint.origin
                link = joint.child

                dh = [0.0, 0.0, 0.0, 0.0]

                for link_elem in self.robot.links:
                    if link_elem.name == link:
                        dh[0] = link_elem.visual.origin.xyz[0]
                        dh[1] = link_elem.visual.origin.xyz[1]
                        dh[2] = link_elem.visual.origin.xyz[2]

                dh[3] = origin.rpy[2]
                
                dh_parameters.append(dh)

        if robot_model:
            robot_model.n = self.n
            robot_model.set_DH_params(dh_parameters)
        else:
            return dh_parameters