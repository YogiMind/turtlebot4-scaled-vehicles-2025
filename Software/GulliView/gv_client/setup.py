from setuptools import setup
from glob import glob
import os

package_name = 'gv_client'  # matches Ros project name
py_package_name = 'gv_client_node'  # matches Python folder name

setup(
    name=package_name,
    version='0.0.0',
    packages=[py_package_name],
    package_dir={'': '.'},  # because gv_client_node is directly inside .
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='your@email.com',
    description='GulliView client node for ROS2',
    license='MIT',
    tests_require=['pytest'],
    data_files=[
        (os.path.join('share', 'gv_client', 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', 'gv_client'), ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'laptop_socket_server = gv_client_node.laptop_socket_server:main',
            'gv_socket_server = gv_client_node.gv_socket_server:main',
            'gv_socket_logger = gv_client_node.gv_socket_logger:main',
        ],
    },
)
