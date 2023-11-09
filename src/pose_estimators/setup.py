from setuptools import find_packages, setup

package_name = 'pose_estimators'

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
            'vo_node = pose_estimators.visual_odometry.vo_node:main',
            'slam_node = pose_estimators.imu_tracker.slam_node:main'
        ],
    },
)
