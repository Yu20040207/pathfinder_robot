from setuptools import setup

setup(name='fwr_navigation', version='0.1.0', packages=['fwr_navigation'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/fwr_navigation']),
                  ('share/fwr_navigation', ['package.xml'])],
      entry_points={'console_scripts': ['path_preview_node = fwr_navigation.node:main',
                                        'navigation_demo_node = fwr_navigation.demo_node:main',
                                        'voice_chassis_demo_node = fwr_navigation.voice_chassis_demo:main']},
      zip_safe=True, maintainer='Robot Team', maintainer_email='robot@example.invalid',
      description='Hardware-independent ROS 2 path preview', license='Apache-2.0')
