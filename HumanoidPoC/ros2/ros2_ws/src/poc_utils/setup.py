from setuptools import setup

package_name = 'poc_utils'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='example@todo.todo',
    description='Utility nodes for PoC simulations (Nav2 status viewer, etc.)',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rviz_republisher = poc_utils.rviz_republisher:main',
            'costmap_update_monitor = poc_utils.costmap_update_monitor:main',
            'pseudo_odometry_node = poc_utils.pseudo_odometry_node:main',
            'imu_odometry_node = poc_utils.imu_odometry_node:main',
            # 'costmap_visualizer = poc_utils.costmap_visualizer:main',
            'nav2_status_viewer = poc_utils.nav2_status_viewer:main',
            'nav2_success_flag = poc_utils.nav2_success_flag:main',
        ],
    },
)
