from setuptools import find_packages, setup

package_name = 'actions_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kevin',
    maintainer_email='kevin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "motor_client = actions_py.motor_client:main",
            "motor_server = actions_py.motor_server:main",
            "propeller_client = actions_py.propeller_client:main",
            "propeller_server = actions_py.propeller_server:main",
            "imu_reading = actions_py.imu_reading:main"
        ],
    },
)
