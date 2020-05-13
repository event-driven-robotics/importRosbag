# Always prefer setuptools over distutils
from setuptools import setup, find_packages
# To use a consistent encoding
from codecs import open
from os import path

here = path.abspath(path.dirname(__file__))

# Get the long description from the README file
with open(path.join(here, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

setup(
    name = "importRosbag",
    version = "0.1.0",
    author = "Sim Bamford",
    description = ("Standalone rosbag loader for python3"),
    long_description=long_description,
)
