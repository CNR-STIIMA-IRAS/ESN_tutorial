from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'esn_tutorial_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='samuele.sandrini@stiima.cnr.it',
    description='ESN tutorial package in python',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'limit_switch_node = esn_tutorial_py.limit_switch_node:main',
            'vision_server_node = esn_tutorial_py.vision_server:main',
            'robot_action_server_node = esn_tutorial_py.robot_action_server_node:main',
            'orchestrator_node = esn_tutorial_py.orchestrator_node:main',
            'test_node = esn_tutorial_py.test_node:main',
        ],
    },
)
