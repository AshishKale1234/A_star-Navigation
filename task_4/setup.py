from setuptools import setup
import os
from glob import glob

package_name = 'task_4'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        # Install map files
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*'))),
        # Install RViz config files
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ashish Kale',
    maintainer_email='kale32@purdue.edu',
    description='SLAM and Navigation for TurtleBot4',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auto_navigator = task_4.auto_navigator:main',
        ],
    },
)
