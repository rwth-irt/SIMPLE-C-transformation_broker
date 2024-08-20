from setuptools import setup

package_name = 'calibration_transformation_broker'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': '.'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dein Name',
    maintainer_email='deine.email@example.com',
    description='ROS2 node to read and extract transformations from a log file and broadcast them as tf2 transformations',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration_transformation_broker = calibration_transformation_broker.calibration_transformation_broker:main'
        ],
    },
)
