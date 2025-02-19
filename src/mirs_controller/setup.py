
from setuptools import setup
import os
from glob import glob

package_name = 'mirs_controller'

setup(
    name=package_name,
    packages=[package_name,
    "mirs_controller/actions",
    "mirs_controller/common",
    "mirs_controller/devices",
    "mirs_controller/dynamics",
    "mirs_controller/hardware",
    "mirs_controller/kinematics",
    "mirs_controller/trajectory",
    "mirs_controller/transformations"],
    version='0.0.1',
    data_files=[
        ('share\\ament_index\\resource_index\\packages',
            ['resource\\' + package_name]),
        ('share\\' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob('launch\\*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chethan Putran',
    maintainer_email='chethanputran222@gmail.com',
    description='MIRS Controller Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "mirs_robot=mirs_controller.robot:main",
        # "mirs_joint_state_publisher=mirs_controller.joint_state_publisher:main"
        ],
    },
)
