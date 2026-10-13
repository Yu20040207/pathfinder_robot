from setuptools import setup

setup(name='fwr_attendance', version='0.1.0', packages=['fwr_attendance'], package_data={'fwr_attendance': ['templates/*.html']}, data_files=[('share/ament_index/resource_index/packages', ['resource/fwr_attendance']), ('share/fwr_attendance', ['package.xml'])], entry_points={'console_scripts': ['attendance_node = fwr_attendance.node:main', 'attendance_web = fwr_attendance.web:main']}, zip_safe=True, maintainer='Robot Team', maintainer_email='robot@example.invalid', description='Face attendance', license='Apache-2.0')
