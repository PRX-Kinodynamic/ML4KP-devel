#! /usr/bin/env python3

from setuptools import setup, find_packages
print(find_packages())
setup(
    name='mujoco_env_creator',
    version='0.1',
    packages=find_packages(),
)