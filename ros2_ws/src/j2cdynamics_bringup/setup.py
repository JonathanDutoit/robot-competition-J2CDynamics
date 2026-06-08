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

        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),

        (os.path.join('share', package_name, 'maps'),
            glob(os.path.join(os.path.dirname(__file__), 'arena', '*.yaml'))),

        (os.path.join('share', package_name, 'config'),
            glob(os.path.join(os.path.dirname(__file__), 'config', '*.yaml'))),

        (os.path.join('share', package_name, 'behavior_trees'),
            glob(os.path.join(os.path.dirname(__file__), 'behavior_trees', '*.xml'))),
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
