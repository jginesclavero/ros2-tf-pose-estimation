#!/usr/bin/env python

from setuptools import find_packages
from setuptools import setup

package_name = 'tf_pose_estimation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Jonatan Gines',
    author_email='jonatan.gines@urjc.es',
    maintainer='Jonatan Gines',
    maintainer_email='jonatan.gines@urjc.es',
    keywords=['ROS2', 'OpenPose', 'TensorFlow'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'The tf_pose_estimation package'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_pose_estimation = tf_pose_estimation.tf_pose_estimation_node:main'
        ],
    },
)
