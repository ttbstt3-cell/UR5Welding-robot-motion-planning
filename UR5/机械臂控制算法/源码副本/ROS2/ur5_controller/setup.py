from setuptools import find_packages, setup

package_name = 'ur5_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ttbstt',
    maintainer_email='ttbstt@todo.todo',
    description='Standalone ROS 2 controller and CoppeliaSim bridge for the UR5.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'controller = ur5_controller.controller:main',
        ],
    },
)
