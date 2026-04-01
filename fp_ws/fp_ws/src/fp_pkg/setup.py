import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'fp_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.pt'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='kemco01@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'map_relay=fp_pkg.map_relay:main',
            'db_astar=fp_pkg.db_astar:main',
            'unloading_node=fp_pkg.unloading_node:main',
            'number_detected_node = fp_pkg.number_detected_node:main',
	        'hey_yang=fp_pkg.hey_yang:main',
            'stopandreturn=fp_pkg.stopandreturn:main',
            'yolo_slow_node=fp_pkg.yolo_slow_node:main',
        ],
    },
)
