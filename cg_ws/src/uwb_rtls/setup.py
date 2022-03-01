from setuptools import setup

package_name = 'uwb_rtls'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='jharring@andrew.cmu.edu',
    description='UltraWideband Real Time Localization System',
    license='N/A',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uwb_rtls = uwb_rtls.uwb_tracking_dwm1001:main'
        ],
    },
)
