from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'brain'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/rviz', ['rviz/tictactoe.rviz']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*.npy')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='blink',
    maintainer_email='z5311131@ad.unsw.edu.au',
    description='Tic-Tac-Toe Brain Package',
    license='TODO',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'brain_node = brain.brain_node:main',
            'keyboard_node = brain.keyboard_node:main',
        ],
    },
)
