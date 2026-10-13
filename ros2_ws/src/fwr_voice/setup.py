from setuptools import setup

setup(name="fwr_voice", version="0.1.0", packages=["fwr_voice"],
      data_files=[("share/ament_index/resource_index/packages", ["resource/fwr_voice"]),
                  ("share/fwr_voice", ["package.xml"])],
      entry_points={"console_scripts": ["voice_node = fwr_voice.node:main",
                                       "voice_demo_executor = fwr_voice.demo_executor:main"]}, zip_safe=True,
      maintainer="Robot Team", maintainer_email="robot@example.invalid",
      description="Voice interaction", license="Apache-2.0")
