
import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'client_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='won',
    maintainer_email='2dongwon@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
                'service_client = client_pkg.service_client:main',
                'motion_client = client_pkg.motion_service_client:main',
                'motion2_client = client_pkg.motion2_service_client:main',
                'keyboard_publisher = client_pkg.keyboard_publisher:main',
                'service_server = client_pkg.service_server:main',
        ],
    },
)
