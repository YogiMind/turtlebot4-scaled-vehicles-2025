from setuptools import setup

package_name = 'gv_client_node'  # matches your Python folder name

setup(
    name='gv_client',
    version='0.1.0',
    packages=[package_name],
    package_dir={'': '.'},  # because gv_client_node is directly inside .
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='your@email.com',
    description='GulliView client node for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'laptop_socket_server = gv_client_node.laptop_socket_server:main',
            'gv_socket_server = gv_client_node.gv_socket_server:main',
            'gv_socket_logger = gv_client_node.gv_socket_logger:main',
        ],
    },
)
