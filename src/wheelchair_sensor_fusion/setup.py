"""Setup file for wheelchair_sensor_fusion package."""

import os
from glob import glob
from setuptools import setup

package_name = 'wheelchair_sensor_fusion'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=[
        'setuptools',
        'numpy>=1.20.0',
        'opencv-python>=4.5.0',
        'scikit-learn>=1.0.0',
        'scipy>=1.7.0',
        'ultralytics>=8.0.0',  # YOLOv11
    ],
    zip_safe=True,
    maintainer='Siddharth Tiwari',
    maintainer_email='s24035@students.iitmandi.ac.in',
    description='Advanced 2D LiDAR and Camera Sensor Fusion for Wheelchair Obstacle Avoidance',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_fusion_node = wheelchair_sensor_fusion.sensor_fusion_node:main',
            'sensor_fusion_node_robust = wheelchair_sensor_fusion.sensor_fusion_node_robust:main',
            'yolo_detector_node = wheelchair_sensor_fusion.yolo_detector_node:main',
            'lidar_processor_node = wheelchair_sensor_fusion.lidar_processor_node:main',
            'obstacle_publisher_node = wheelchair_sensor_fusion.obstacle_publisher_node:main',
        ],
    },
)
