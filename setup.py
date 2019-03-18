# DO NOT USE python setup.py install

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['planner_tests'],
    scripts=[],
    package_dir={'': 'python/src'},
    requires=['rospy', 'numpy', 'geometry_msgs', 'itertools']
)

setup(**d)
