from setuptools import setup
import os
from glob import glob

main_package = 'training_py'
robot_simulation_pkg = main_package + "/robot_simulation_pkg"

setup(
    name=main_package,
    version='0.0.0',
    packages=[main_package,robot_simulation_pkg],
    # Files we want to install, specifically launch files
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + main_package]),
        # Include our package.xml file
        ('share/' + main_package, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', main_package, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        # Include all config files
        (os.path.join('share', main_package, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alex',
    maintainer_email='alexis-h-34@hotmail.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        # nom_du_noeud = nom_folder.nomfufichiersource:main
        'console_scripts': [
            'get_expert_pose_node = training_py.get_expert_pose:main',
            'test_yaml_node = training_py.testyaml:main'
        ],
    },
)
