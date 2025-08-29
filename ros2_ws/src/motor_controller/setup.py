from setuptools import setup

package_name = 'motor_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','pyserial'],
    zip_safe=True,
    maintainer='project',
    maintainer_email='baptistepnct@gmail.com',
    description='ROS2 package for controlling motors via serial communication',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_node = motor_controller.motor_node:main'
        ],
    },
)
