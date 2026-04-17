from setuptools import find_packages, setup


package_name = 'perception_yolo'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Juan Millan',
    maintainer_email='jmillan1997@gmail.com',
    description='WSL-side YOLO perception node for camera-based detection and person recognition.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'yolo_detector = perception_yolo.yolo_detector:main',
        ],
    },
)
