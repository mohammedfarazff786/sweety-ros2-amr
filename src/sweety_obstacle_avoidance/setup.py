from setuptools import find_packages, setup

package_name = 'sweety_obstacle_avoidance'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mohammedfarazff786',
    maintainer_email='mohammedfarazff786@example.com',
    description='DWA-based obstacle avoidance and local path planning for Sweety AMR',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoidance_node=sweety_obstacle_avoidance.obstacle_avoidance_node:main',
        ],
    },
)
