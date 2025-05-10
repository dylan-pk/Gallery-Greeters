from setuptools import find_packages
from setuptools import setup

setup(
    name='audio_messages',
    version='0.0.0',
    packages=find_packages(
        include=('audio_messages', 'audio_messages.*')),
)
