## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
# https://www.youtube.com/watch?v=9bgrzZ45HMA
setup_args = generate_distutils_setup(
    packages=['aidan_homunculus'],
    package_dir={'': 'src'},
)

setup(**setup_args) 