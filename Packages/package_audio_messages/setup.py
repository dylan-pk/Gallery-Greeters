from setuptools import setup
import os
from glob import glob

package_name = 'package_audio_messages'

def package_files(directory):
    paths = []
    for path in glob(os.path.join(directory, "*")):
        if os.path.isfile(path):
            paths.append(os.path.relpath(path, start=os.getcwd()))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resources', package_files('package_audio_messages/resources')),
        ('share/' + package_name + '/resources/ArtworkInfo', package_files('package_audio_messages/resources/ArtworkInfo')),
        ('share/' + package_name + '/resources/FunFacts', package_files('package_audio_messages/resources/FunFacts')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anika',
    maintainer_email='anika.roth@student.uts.edu.au',
    description='Voice Command Module for Gallery Greeters Project',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speech_node = package_audio_messages.SpeechToText:main',
        ],
    }
)

# from setuptools import find_packages, setup

# package_name = 'audio_messages'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='anika',
#     maintainer_email='anika.roth@student.uts.edu.au',
#     description='Voice Command Module for Gallery Greeters Project',
#     license='MIT',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#         ],
#     },
# )
