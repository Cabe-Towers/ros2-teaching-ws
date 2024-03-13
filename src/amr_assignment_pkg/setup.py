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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cabe Towers',
    maintainer_email='cabe.towers@gmail.com',
    description='Autonomous mobile robotics assignment package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'assignment_node = amr_assignment_pkg.assignment_node:main',
        ],
    },
)
