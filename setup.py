from setuptools import find_packages, setup
from glob import glob

package_name = 'tripteron'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/rviz',   glob('rviz/*')),
        ('share/' + package_name + '/urdf',   glob('urdf/*')),
        ('share/' + package_name + '/tripteron',   glob('tripteron/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='The 133a Project Code Demos',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'boolean_publisher = tripteron.boolean_publisher:main',
            'float_publisher   = tripteron.float_publisher:main',
            'point_publisher   = tripteron.point_publisher:main',
            'tripmove          = tripteron.tripmove:main',
        ],
    },
)
