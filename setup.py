# Always prefer setuptools over distutils
from setuptools import setup
# To use a consistent encoding
from os import path

here = path.abspath(path.dirname(__file__))

# Get the long description from the README file
with open(path.join(here, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

setup(
    name = "importRosbag",
    version = "1.0.3",
    author = "Sim Bamford",
    description = ("Standalone rosbag loader for python3"),
    packages=['importRosbag', 'importRosbag.messageTypes'],
    license='gpl',
    long_description=long_description,
    long_description_content_type='text/markdown',
    author_email = 'simbamford@gmail.com',
    url = 'https://github.com/event-driven-robotics/importRosbag',
    download_url = 'https://github.com/event-driven-robotics/importRosbag/archive/v1.0.tar.gz',
    keywords = ['ros', 'rosbag', 'bag', 'rpg', 'dvs', 'rpg_dvs_ros', 'event', 'event camera', 'event-based', 'event-driven', 'dynamic vision sensor', 'neuromorphic', 'aer', 'address-event representation' 'spiking neural network', 'davis', 'atis', 'celex' ],
    install_requires=[
        'numpy',
        'tqdm',
        'setuptools',
      ],
  classifiers=[
    'Development Status :: 3 - Alpha',
    'Intended Audience :: Developers',
    'Topic :: Software Development :: Build Tools',
    'License :: OSI Approved :: GNU General Public License (GPL)',
    'Programming Language :: Python :: 3',
    'Programming Language :: Python :: 3.7',
  ],
)
