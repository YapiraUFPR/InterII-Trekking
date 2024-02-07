from setuptools import find_packages, setup

package_name = 'drivers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),        
        ('share/' + package_name, ['launch/drivers_launch.py']),
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
            'camera_publisher = drivers.camera_publisher:main',
            'color_publisher = drivers.tcs32_publisher:main',
            'imu_publisher = drivers.bno_publisher:main',
            'irdist_publisher = drivers.irdist_publisher:main',
            'battery_publisher = drivers.battery_publisher:main',
        ],
    },
)
