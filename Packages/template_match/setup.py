from setuptools import setup
import os
from glob import glob

package_name = 'template_match'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # the subdirectory that contains __init__.py + template_match.py
    data_files=[
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/resource', glob('resource/*')),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/camera_info', glob('camera_info/*')),
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
            'camera_node = template_match.simple_cam_node:main',
        ],
    },

)
