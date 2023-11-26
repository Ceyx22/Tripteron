from setuptools import find_packages, setup
from glob import glob

package_name = 'Tripteron'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/rviz',   glob('rviz/*')),
        ('share/' + package_name + '/Description',   glob('Description/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FM',
    maintainer_email='robot@todo.todo',
    description='Tripteron Demo',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'boolean_publisher = Util.boolean_publisher:main',
            'float_publisher   = Util.float_publisher:main',
            'point_publisher   = Util.point_publisher:main',
        ],
    },
)
