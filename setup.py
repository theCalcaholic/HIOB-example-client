from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


d = generate_distutils_setup(
    packages=[
        'app',
        'app.ros',
        'app.ui'],
    package_dir={'': './'}
)

setup(**d)