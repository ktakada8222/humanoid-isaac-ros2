from setuptools import setup

package_name = 'poc_failsafe'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Failsafe node that stops Nav2 when LIDAR stops publishing.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lidar_failsafe_node = poc_failsafe.lidar_failsafe_node:main',
            'obstacle_failsafe_node = poc_failsafe.obstacle_failsafe_node:main',
        ],
    },
)
