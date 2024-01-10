from setuptools import setup
import os

package_name = 'rtcm_provider'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, [os.path.join('config', 'korea_rtcm_base.yaml')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ha Trung',
    maintainer_email='nguyenhatrung411@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rtcm_ntrip_pub = rtcm_provider.rtcm_ntrip_pub:main',
        ],
    },
)
