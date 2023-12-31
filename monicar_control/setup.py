import os
from glob import glob
from setuptools import find_packages
from setuptools import setup

package_name = 'monicar_control'
submodules = "monicar_control/submodules"

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'param'), glob('param/*.yaml')),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    author='ChangWhan Lee',
    author_email='zeta0707@gmail.com',
    maintainer='ChangWhan Lee',
    maintainer_email='zeta0707@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Teleoperation node using keyboard or joystick for the Monicar'
    ),
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'blob_chase = monicar_control.blob_chase:main',
            'chase_the_ball = monicar_control.chase_the_ball:main', 
            'joy_control = monicar_control.joy_control:main',
            'motor_control = monicar_control.motor_control:main',
            'chase_object_yolo = monicar_control.chase_object_yolo:main', 
            'chase_traffic_yolo = monicar_control.chase_traffic_yolo:main', 
        ],
    },
)
