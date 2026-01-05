from setuptools import setup, find_packages

package_name = 'sweety_slam'

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
    maintainer='Mohammed Faraz',
    maintainer_email='mohammedfarazff786@example.com',
    description='SLAM implementation for Sweety ROS2 AMR',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam_node = sweety_slam.slam_node:main',
        ],
    },
)
