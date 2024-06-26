from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'amr_assignment_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Allows config and util files to be imported by code
        ('lib/' + package_name, [package_name + '/config.py']),
        ('lib/' + package_name, [package_name + '/util.py']),
        ('lib/' + package_name, [package_name + '/pathfinding.py']),
    ],
    install_requires=['setuptools', 'scipy', 'simple-pid'],
    zip_safe=True,
    maintainer='Cabe Towers',
    maintainer_email='cabe.towers@gmail.com',
    description='Autonomous mobile robotics assignment package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_node = amr_assignment_pkg.state_node:main',
            'depth_camera_node = amr_assignment_pkg.depth_camera_node:main',
            'global_reference_frame = amr_assignment_pkg.global_reference_frame:main',
            'nav_node = amr_assignment_pkg.nav_node:main',
            'simple_node = amr_assignment_pkg.simple_state_machine:main'
        ],
    },
)
