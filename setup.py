from setuptools import find_packages, setup
from glob import glob

package_name = 'demos'

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
            'boolean_publisher = demos.boolean_publisher:main',
            'float_publisher   = demos.float_publisher:main',
            'point_publisher   = demos.point_publisher:main',
            'balldemo          = demos.balldemo:main',
            'fencedemo         = demos.fencedemo:main',
            'interactivedemo   = demos.interactivedemo:main',
            'pirouette         = demos.pirouette:main',
            'pirouetteandwave  = demos.pirouetteandwave:main',
        ],
    },
)
