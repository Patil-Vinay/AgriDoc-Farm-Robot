from glob import glob

from setuptools import setup

package_name = 'agridoc_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vinay Patil',
    maintainer_email='patilb.vinay@gmail.com',
    description='Mapping and autonomous navigation for the AgriDoc robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'patrol = agridoc_navigation.patrol:main',
        ],
    },
)
