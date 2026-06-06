from setuptools import setup
from glob import glob
import os

package_name = 'j2cdynamics_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jed',
    maintainer_email='dutoit.jonathaneric@gmail.com',
    description='Ground-plane projection + clustered duplo map.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ground_projection = j2cdynamics_perception.ground_projection:main',
            'duplo_map = j2cdynamics_perception.duplo_map:main',
        ],
    },
)
