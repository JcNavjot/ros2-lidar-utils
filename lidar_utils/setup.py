from setuptools import setup

package_name = 'lidar_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jot',
    maintainer_email='jot@todo.todo',
    description='Universal LiDAR utilities and inspection tools',
    license='MIT',
    entry_points={
        'console_scripts': [
            'laser_inspector = lidar_utils.laser_inspector:main',
            'sector_debugger = lidar_utils.sector_debugger:main',
        ],
    },
)

