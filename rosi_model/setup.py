from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rosi_model'],
    package_dir={'': 'src'}
)

setup(**d)