from setuptools import find_packages, setup

package_name = 'esn_tutorial_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'vision_system_node = esn_tutorial_py.vision_system_node:main',
            # 'robot_action_server = esn_tutorial_py.robot_action_server:main',
            'test_node = esn_tutorial_py.test_node:main',
        ],
    },
)
