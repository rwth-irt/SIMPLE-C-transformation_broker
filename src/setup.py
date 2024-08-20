from setuptools import find_packages, setup

package_name = 'calibration_transformation_broker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='timrehbronn',
    maintainer_email='tim.rehbronn@rwth-aachen.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration_transformation_broker = calibration_transformation_broker.calibration_transformation_broker:main'
        ],
    },
)
