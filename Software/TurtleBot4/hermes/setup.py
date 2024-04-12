from setuptools import setup
from glob import glob
import os
package_name = 'hermes'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*launch.py*'))),
        ('share/' + package_name + '/config', glob(os.path.join('config', '*yaml*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [            
            'hermes_core = hermes.hermes_core:main',
            'depth_intensity_image_syncer = hermes.depth_intensity_image_syncer:main',
        ],
    },
)
