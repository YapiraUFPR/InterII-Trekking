from setuptools import setup

package_name = 'visual_pid'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='milena',
    maintainer_email='milafrancis2016@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cone_detector = visual_pid.cone_detector:main',
            '2d_pid = visual_pid.2d_pid:main',
        ],
    },
)
