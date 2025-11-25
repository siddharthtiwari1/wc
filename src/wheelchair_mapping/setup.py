from setuptools import find_packages, setup

package_name = 'wheelchair_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/slam_toolbox_v14r25_FINAL.yaml',
            'config/slam_toolbox_s3_max.yaml',
            'config/slam_toolbox_cluttered.yaml',
            'config/slam_toolbox_no_dots.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/wheelchair_slam_mapping.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='siddharth',
    maintainer_email='s24035@students.iitmandi.ac.in',
    description='Wheelchair SLAM mapping package with SLAM Toolbox',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
