
from setuptools import setup
import os
from glob import glob


package_name = 'mirs_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch/'),glob('launch/*launch.[pxy][yma]*')),
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
        "robot=mirs_controller.robot:main"
            "joint_state_publisher = mirs_controller.joint_state_publisher:main",
            "robot_state_publisher = mirs_controller.robot_state_publisher:main",
            "trajectory_follower = mirs_controller.trajectory_follower:main",
        ],
    },
)
