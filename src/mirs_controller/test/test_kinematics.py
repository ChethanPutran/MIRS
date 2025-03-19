import numpy as np
from mirs_controller.kinematics.kinematics import Kinematics
from mirs_controller.transformations.transform import Transform
from mirs_controller.common.urdf_converter import URDFConverter

def test_kinematics_jacob():
    robot = URDFConverter(TEST_MODEL=True).get_robot_model()
    transform = Transform(robot)
    kinematics = Kinematics(transform)
    q_dot = np.array([1, 1, 1]).reshape(-1, 1)

    J = kinematics.jacobian()
    O = np.zeros((6, robot.n))

    assert (J.shape[0] == O.shape[0]) and (J.shape[1] == O.shape[1])
