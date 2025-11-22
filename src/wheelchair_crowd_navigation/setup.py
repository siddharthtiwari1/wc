from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'wheelchair_crowd_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # Model files
        (os.path.join('share', package_name, 'models'),
            glob('models/*.pt*')),
    ],
    install_requires=[
        'setuptools',
        'torch>=2.0.0',
        'jax[cuda12]>=0.4.0',
        'flax>=0.8.0',
        'optax>=0.2.0',
        'numpy>=1.24.0',
        'scipy>=1.11.0',
        'pyyaml>=6.0',
        'tensorboard>=2.15.0',
        'open3d>=0.18.0',
        'hydra-core>=1.3.2',
    ],
    zip_safe=True,
    maintainer='Siddharth Tiwari',
    maintainer_email='s24035@students.iitmandi.ac.in',
    description='CrowdSurfer-based crowd navigation for wheelchairs',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crowd_planner_node = wheelchair_crowd_navigation.ros2.crowd_planner_node:main',
            'visualization_node = wheelchair_crowd_navigation.ros2.visualization_node:main',
            'train_vqvae = wheelchair_crowd_navigation.training.trainer:main',
            'collect_data = wheelchair_crowd_navigation.training.data_collector:main',
            'evaluate = wheelchair_crowd_navigation.training.evaluator:main',
        ],
    },
)
