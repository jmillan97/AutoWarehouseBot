from setuptools import find_packages, setup

package_name = 'summon'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
        ['launch/summon.launch.py']),
        ('share/' + package_name + '/config',
            ['config/named_locations.yaml',
            'config/landmarks.yaml']),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Juan Millan',
    maintainer_email='jmillan1997@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'summon_node = summon.summon_node:main',
            'summon_server = summon.summon_server:main',
            'ble_tracker = summon.ble_tracker:main',
            'aruco_node = summon.aruco_node:main'
        ],
    },
)
