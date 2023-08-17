from setuptools import setup

package_name = 'workshop'

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
    maintainer='gnhn',
    maintainer_email='ielnascarella@hotmail.com',
    description='Package description',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = workshop.publisher:main',
            'listener = workshop.listener:main',

            'd_publisher = workshop.different_publisher:main',
            'd_listener = workshop.different_listener:main',
            
            't_publisher = workshop.timer_publisher:main',
            't_listener = workshop.timer_listener:main',

            'wanderbot = workshop.wanderbot:main'
        ],
    },
)
