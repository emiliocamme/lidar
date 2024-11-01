from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'urdf_tutorial_r2d2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name), glob('launch/*launch.[pxy][yam]*')),
        (os.path.join('share',package_name), glob('launch/*.[pxy][yam]*')),
        (os.path.join('share',package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emitaker',
    maintainer_email='emitaker@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'static = urdf_tutorial_r2d2.static:main',
        'dynamic = urdf_tutorial_r2d2.dynamic:main',
        'lidar_publisher = urdf_tutorial_r2d2.lidar_publisher:main',
        'lidar_publisher_movement = urdf_tutorial_r2d2.lidar_publisher_movement:main',
        'lidar_publisher_slam = urdf_tutorial_r2d2.lidar_publisher_slam:main'
        ],
    },
)
