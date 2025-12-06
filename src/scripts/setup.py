from setuptools import find_packages, setup

package_name = 'scripts'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Removed redundant launch files - all testing is now done via main launch files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='diadem',
    maintainer_email='diadem@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        # Core utilities - KEEP
        'twist_stamped_teleop = scripts.twist_stamped_teleop:main',
        'imu_out_to_imu = scripts.imu_out_to_imu:main',
        'sim_odom_bias = scripts.sim_odom_bias:main',
        'topic_data_logger = scripts.topic_data_logger:main',
        'mapping_data_logger = scripts.mapping_data_logger:main',
        # Active test scripts - KEEP
        'square_path_ekf_tester = scripts.square_path_ekf_tester:main',
        'l_shape_odometry_test.py = scripts.l_shape_odometry_test:main',
        'square_odometry_test_enhanced.py = scripts.square_odometry_test_enhanced:main',
        # Data analysis
        'analyze_imu_odometry_log = scripts.analyze_imu_odometry_log:main',
        ],
    },
)
