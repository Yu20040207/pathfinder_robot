from setuptools import setup


setup(
    name='fwr_imu',
    version='0.1.0',
    packages=['fwr_imu'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/fwr_imu']),
        ('share/fwr_imu', ['package.xml']),
    ],
    entry_points={'console_scripts': [
        'dm_imu_node = fwr_imu.node:main',
        'imu_preview_tf = fwr_imu.preview:main',
    ]},
    zip_safe=True,
    maintainer='Robot Team',
    maintainer_email='robot@example.invalid',
    description='DM-IMU-L1 USB ROS 2 driver',
    license='Apache-2.0',
)
