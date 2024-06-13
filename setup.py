from setuptools import find_packages, setup

package_name = 'smallsensor_test'

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
    maintainer='roma-lab',
    maintainer_email='roma-lab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_old_ros2 = smallsensor_test.sensor_old_ros2:main',
            'sensor_five = smallsensor_test.sensor_five:main',
            'motor_con = smallsensor_test.motor_con:main',
            'motor_publisher = smallsensor_test.motor_publisher:main',
            'motor_subscriber = smallsensor_test.motor_subcriber:main'
        ],
    },
)
