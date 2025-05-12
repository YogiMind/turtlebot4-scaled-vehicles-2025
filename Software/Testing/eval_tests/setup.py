from setuptools import find_packages, setup

package_name = 'eval_tests'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='scazard',
    maintainer_email='eliassamuelsson321@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'pose_controller = eval_tests.pose_controller:main',
                'pose_recorder = eval_tests.pose_recorder:main',
        ],
    },
)
