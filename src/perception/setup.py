from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include meshes
        (os.path.join('share', package_name, 'meshes'), 
            glob('meshes/*.stl')),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='blink',
    maintainer_email='z5311131@ad.unsw.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aruco_vision_node = perception.aruco_vision:main',
            'cell_vision_node = perception.cell_vision:main',
            'publish_grid_pose = perception.publish_grid_pose:main',
        ],
    },
)
