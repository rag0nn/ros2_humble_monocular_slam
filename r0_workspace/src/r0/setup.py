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
            'data_provider = r0.data_provider:main'
        ],
    },
)
