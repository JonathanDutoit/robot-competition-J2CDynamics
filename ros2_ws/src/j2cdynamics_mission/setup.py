from setuptools import setup

package_name = 'j2cdynamics_mission'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jed',
    maintainer_email='dutoit.jonathaneric@gmail.com',
    description='Competition mission runner for the J2C-Dynamics robot.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mission_runner = j2cdynamics_mission.mission_runner:main',
            'explore_zone = j2cdynamics_mission.explore_zone:main',
            'da_mission = j2cdynamics_mission.da_mission:main',
            'do_mission = j2cdynamics_mission.do_mission:main',
        ],
    },
)
