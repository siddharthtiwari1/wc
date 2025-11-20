from setuptools import find_packages, setup

package_name = 'wheelchair_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/ekf.yaml',
            'config/ekf_global.yaml',
            'config/slam_toolbox.yaml',
            'config/slam_toolbox_mapping.yaml',
            'config/slam_toolbox_s3_cluttered_indoor.yaml',
            'config/slam_toolbox_s3_aggressive.yaml',
            'config/slam_toolbox_s3_optimized.yaml',
            'config/slam_toolbox_s3_pro.yaml',
            'config/slam_toolbox_s3_white_walls.yaml',
            'config/slam_toolbox_wheelchair_optimized.yaml',
            'config/slam_toolbox_high_quality.yaml',
            'config/amcl.yaml',
            'config/amcl_wheelchair_optimized.yaml'
        ]),
        ('share/' + package_name + '/launch', [
            'launch/local_localization.launch.py',
            'launch/simplified_localization.launch.py',
            'launch/wheelchair_localization.launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='siddharth',
    maintainer_email='siddharth@todo.todo',
    description='Wheelchair localization package with EKF filter',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # All republishers removed - functionality handled by wc_control/imu_wheelchair_republisher
            # Removed: imu_to_base_republisher, camera_odom_republisher, simple_imu_republisher
        ],
    },
)
