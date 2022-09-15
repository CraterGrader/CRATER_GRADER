from setuptools import setup
from glob import glob
import os

package_name = 'total_station'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='jharring@andrew.cmu.edu',
    description='Leica Total Station (TS16) Real Time Localization System',
    license='N/A',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'total_station_node = total_station.total_station_node:main'
        ],
    },
)
