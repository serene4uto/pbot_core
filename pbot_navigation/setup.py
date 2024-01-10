from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pbot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='nguyenhatrung411@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interactive_waypoint_follower = pbot_navigation.interactive_waypoint_follower:main',
            'gps_waypoint_logger = pbot_navigation.gps_waypoint_logger:main',
            'logged_waypoint_follower = pbot_navigation.logged_waypoint_follower:main',
            'interactive_gps_waypoint_logger = pbot_navigation.interactive_gps_waypoint_logger:main',
        ],
    },
)
