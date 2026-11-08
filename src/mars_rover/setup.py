from glob import glob
from setuptools import setup

package_name = 'mars_rover'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rover_dev',
    maintainer_email='rover@example.com',
    description='Mars rover simulation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'obstacle_avoider = mars_rover.obstacle_avoider:main',
            'cmd_vel_mux = mars_rover.cmd_vel_mux:main',
            'data_server = mars_rover.data_server:main',
            'set_mode = mars_rover.set_mode:main',
            'generate_world = mars_rover.generate_world:main',
        ],
    },
)
