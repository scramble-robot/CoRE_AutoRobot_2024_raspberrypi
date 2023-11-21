from setuptools import setup

package_name = 'scramble_auto_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/hardware.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='CoRE competition committee',
    author_email='info-core@scramble-robot.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Nodes of CoRE automatic-robot.',
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'can_node = scramble_auto_robot.can_node:main',
            'servo_node = scramble_auto_robot.servo_node:main',
            'gpio_node = scramble_auto_robot.gpio_node:main',
        ],
    },
)
