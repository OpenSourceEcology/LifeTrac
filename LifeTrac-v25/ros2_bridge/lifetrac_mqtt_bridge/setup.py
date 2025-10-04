from setuptools import setup
from glob import glob
import os

package_name = 'lifetrac_mqtt_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Open Source Ecology',
    maintainer_email='info@opensourceecology.org',
    description='ROS2 to MQTT bridge for LifeTrac v25 control system',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mqtt_bridge_node = lifetrac_mqtt_bridge.mqtt_bridge_node:main',
            'test_publisher = lifetrac_mqtt_bridge.test_publisher:main',
        ],
    },
)
