from setuptools import setup
import os
from glob import glob


package_name = 'mirs_system'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name,
    package_name+"\\conf",
    package_name+"\\ai",
    package_name+"\\ai\\vision",
    package_name+"\\ai\\task",
    package_name+"\\ai\\voice",
    ],
    data_files=[
        ('share\\ament_index\\resource_index\\packages',
            ['resource\\' + package_name]),
        ('share\\' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob('launch\\*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chethan_putran',
    maintainer_email='chethansputran222@gmail.com',
    description='MIRS system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "voice = mirs_system.ai.voice.voice:main", 
        "system = mirs_system.system:main",
        "task_recorder = mirs_system.task_recorder:main",
        "task_extractor = mirs_system.task_extractor:main",
        "task_executor = mirs_system.task_executor:main", 
        ],
    },
)
