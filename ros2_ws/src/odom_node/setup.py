from setuptools import setup

package_name = 'odom_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='project',
    maintainer_email='baptistepnct@gmail.com',
    description='Node to compute odometry from CAN messages',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'odom_node = odom_node.odom_node:main',
        ],
    },
)
