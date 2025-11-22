from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'wheelchair_adaptive_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'),
            glob('models/*.pt*')),
    ],
    install_requires=[
        'setuptools',
        'torch>=2.0.0',
        'diffusers>=0.25.0',
        'accelerate>=0.26.0',
        'numpy>=1.24.0',
        'scipy>=1.11.0',
        'scikit-learn>=1.3.0',
        'filterpy>=1.4.5',
        'pyyaml>=6.0',
        'tensorboard>=2.15.0',
    ],
    zip_safe=True,
    maintainer='Siddharth Tiwari',
    maintainer_email='s24035@students.iitmandi.ac.in',
    description='Diffusion-based adaptive navigation with social awareness',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'adaptive_planner_node = wheelchair_adaptive_navigation.ros2.adaptive_planner_node:main',
            'social_viz_node = wheelchair_adaptive_navigation.ros2.social_viz_node:main',
            'train_diffusion = wheelchair_adaptive_navigation.training.diffusion_trainer:main',
            'evaluate = wheelchair_adaptive_navigation.training.evaluator:main',
        ],
    },
)
