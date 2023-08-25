from setuptools import setup
import os
from glob import glob

package_name = 'part1_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tom Howard',
    maintainer_email='t.howard@sheffield.ac.uk',
    description='Exercises from Part 1 of the COM2009 ROS2 Course',
    license='GNU GPL 3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = part1_pubsub.publisher:main',
            'subscriber = part1_pubsub.subscriber:main'
        ],
    },
)
