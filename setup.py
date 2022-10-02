from setuptools import setup
from pathlib import Path

setup(
    name='drb_module',
    version='0.0.1',
    description="A PyBullet-Gym Environment for Humanoid Stability Training",
    packages=setuptools.find_packages(include="drb_module*"),
    # install_requires=['gym']  # And any other dependencies foo needs
)
