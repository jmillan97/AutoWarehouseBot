import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tailscale_relay'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'websockets'],
    zip_safe=True,
    maintainer='Juan Millan',
    maintainer_email='jmillan1997@email.com',
    description='WebSocket relay for ROS2 topics over Tailscale VPN',
    license='MIT',
    entry_points={
        'console_scripts': [
            'relay_server = tailscale_relay.relay_server:main',
            'relay_client = tailscale_relay.relay_client:main',
        ],
    },
)
