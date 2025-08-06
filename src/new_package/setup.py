from setuptools import find_packages, setup

package_name = 'new_package'

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
    maintainer='hacker',
    maintainer_email='hacker@todo.todo',
    description='First package created for project',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	'hello = new_package.hello:main',
	'DHT = new_package.sensors.dht_data_publisher:main',
    'soilData = new_package.sensors.soil_data_publisher:main',
    'waterFlow = new_package.sensors.waterFlow_publisher:main',
    'subscriber = new_package.subscribe_data:main',
    'rest = new_package.RestApi:main',
    'servo_call = new_package.services.servo_service:main',
    'valve_call = new_package.services.valve_service:main',
    'pump_call = new_package.services.pump_service:main',
        ],
    },
)
