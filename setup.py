## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['writing3d'],
    package_dir={'': 'src'},
    requires=['std_msgs', 'rospy', 'roscpp']
)

setup(**setup_args)
