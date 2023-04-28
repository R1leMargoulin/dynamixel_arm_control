from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['dynamixel_arm_control'],
    package_dir={'': 'scripts'}
)
setup(**d)