#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['am_extruder_simple_ui'],
    package_dir={'': 'src'},
    scripts=['scripts/rqt_image_view']
)

setup(**d)
