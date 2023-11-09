from setuptools import find_packages, setup

package_name = 'sensor_drivers'

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
    maintainer='gab',
    maintainer_email='gabriel.pontarolo045@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = sensor_drivers.camera_publisher:main',
            'color_publisher = sensor_drivers.color_publisher:main',
            'imu_publisher = sensor_drivers.imu_publisher:main',
            'irdist_publisher = sensor_drivers.irdist_publisher:main',
            'battery_publisher = sensor_drivers.battery_publisher:main',
        ],
    },
)
