from setuptools import find_packages, setup

package_name = 'my_package'

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
    maintainer='tahir',
    maintainer_email='tahir@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "py_node = my_package.helloWorld_node:main",
            "robot_news = my_package.robot_news:main",
            "smartphone = my_package.smartphone:main",
            "sum_two_ints = my_package.sum_two_ints:main",
            "sum_two_ints_client=my_package.sum_two_ints_client:main",
            "hw_status_publisher = my_package.hw_status_publisher:main",
            "hw_status_subscriber = my_package.hw_status_subscriber:main",
            "compute_area_server = my_package.compute_area_server:main"
        ],
    },
)
