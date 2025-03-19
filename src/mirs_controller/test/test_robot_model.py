from mirs_controller.common.urdf_converter import URDFConverter

def test_robot_urdf():
    robot = URDFConverter(TEST_MODEL=True).get_robot_model()