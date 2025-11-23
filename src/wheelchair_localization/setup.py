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
            # EKF sensor fusion configs
            'config/ekf.yaml',
            'config/ekf_global.yaml',
            # SLAM Toolbox mapping config (PRODUCTION - SOURCE CODE VERIFIED)
            'config/slam_toolbox_v14r25_FINAL.yaml',
            # AMCL localization configs
            'config/amcl.yaml',
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
