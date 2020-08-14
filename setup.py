# Copyright 2020, Stefano Dell'Orto
# License: BSD
#   https://raw.githubusercontent.com/allxone/sensehat_ros/master/LICENSE
#
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['sensehat_ros'],
    package_dir={'': 'src'},
)

setup(**setup_args)