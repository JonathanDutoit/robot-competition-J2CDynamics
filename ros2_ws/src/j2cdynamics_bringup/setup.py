from setuptools import setup
from glob import glob 
import os

package_name = 'j2cdynamics_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', 'j2cdynamics_bringup', 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', 'j2cdynamics_bringup', 'config'), glob('config/*.yaml')),
        (os.path.join('share', 'j2cdynamics_bringup', 'behavior_trees'), glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jed',
    maintainer_email='dutoit.jonathaneric@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
        ],
    },
)
