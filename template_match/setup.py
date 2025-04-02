from setuptools import setup
import os
from glob import glob

package_name = 'template_match'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
    ('resource/' + package_name, ['package.xml']),
    ('resources/' + package_name + '/resource', glob('resources/*')),  # Image files in resource
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dylan',
    maintainer_email='dylan.m.purbrick-1@student.uts.edu.au',
    description='Template matching package for ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'match_node = template_match:main',
        ],
    },
)
