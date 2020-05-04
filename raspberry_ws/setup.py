## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['wizzybug_control', 'wizzybug_decision_making', 'wizzybug_hmi', 'wizzybug_lidar'],
    package_dir={'': 'src'}
)

setup(**setup_args)
