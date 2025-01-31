from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'r0'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))), # launch files
        ('share/' + package_name + '/config', glob('config/*.rviz')), # rviz config
        ('share/ament_index/resource_index/packages',['resource/' + package_name]), # default
        ('share/' + package_name, ['package.xml']), #default
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rag0n',
    maintainer_email='enes_trhn_8@outlook.com',
    description='TODO: Package description',
    license='Apache2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'provider_rgbd_via_monocular = r0.data_provider_rgbd_via_monocular:main',
            'provider_rgbd = r0.data_provider_rgbd:main',

            'listener_caminfo = r0.data_listener_camera_info:main',
            'listener_depth = r0.data_listener_depth:main',
            'listener_rgb = r0.data_listener_rgb:main',

            'data_recorder= r0.data_recorder:main',
            'fake_clock_publisher = r0.fake_clock_publisher:main',
        ],
    },
)
