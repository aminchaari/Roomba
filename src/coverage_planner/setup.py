from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'coverage_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install all launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Roomba coverage path planning pipeline',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Main coverage planner node
            'coverage_planner = coverage_planner.coverage_node:main',
            # Pipeline manager (monitors exploration, saves map, triggers CPP)
            'pipeline_manager = coverage_planner.pipeline_manager:main',
        ],
    },
)