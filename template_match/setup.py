from setuptools import setup
import os
from glob import glob

package_name = 'template_match'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Required for ROS 2 package discovery via ament_index
        (f'share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        
        # Required to install your package metadata
        (f'share/{package_name}', ['package.xml']),

        # Include everything inside the 'resource' folder (like template.jpg)
        (f'share/{package_name}/resource', glob('resource/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dylan',
    maintainer_email='dylan@example.com',
    description='A ROS 2 package for template matching using OpenCV.',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'template_match = template_match.template_match:main',
        ],
    },
)
