import os
from glob import glob
from setuptools import setup

package_name = 'my_cubes_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name]
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stephenq0503',
    maintainer_email='stephenqiao123@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['reader_mark_node = my_cubes_pkg.coordinate_node:main',
                            'my_client_node = my_sphere_pkg.client_node:main',
        ],
    },
)
