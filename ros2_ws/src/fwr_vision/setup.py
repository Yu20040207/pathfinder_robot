from setuptools import setup

setup(name='fwr_vision', version='0.1.0', packages=['fwr_vision'], data_files=[('share/ament_index/resource_index/packages', ['resource/fwr_vision']), ('share/fwr_vision', ['package.xml'])], entry_points={'console_scripts': ['face_node = fwr_vision.node:main']}, zip_safe=True, maintainer='Robot Team', maintainer_email='robot@example.invalid', description='RGB-D face perception', license='Apache-2.0')
