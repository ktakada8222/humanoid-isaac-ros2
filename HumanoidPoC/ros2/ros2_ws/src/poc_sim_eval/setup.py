from setuptools import setup
import os
from glob import glob

package_name = 'poc_sim_eval'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='',
    maintainer_email='atsuki.akamisaka@tron.tokyo',
    description='Simulation evaluation tools for PoC in Isaac Sim + Nav2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav2_auto_eval = poc_sim_eval.nav2_auto_eval:main',
        ],
    },
)
