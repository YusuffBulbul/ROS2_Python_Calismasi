from setuptools import find_packages, setup


package_name = 'my_py_pkg'

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
    maintainer='yusuf',
    maintainer_email='yusufbulbul475@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "my_py_node = my_py_pkg.my_first_node:main",
            "state_publisher = my_py_pkg.robot_state_publisher:main",
            "satellite = my_py_pkg.satellite:main",
            "add_two_ints_server = my_py_pkg.add_two_ints_server:main",
            "add_two_ints_client_no_oop = my_py_pkg.add_two_ints_client_no_oop:main",
            "add_two_ints_client_oop = my_py_pkg.add_two_ints_client_oop:main",
            "number_publisher = my_py_pkg.number_publisher:main",
            "number_counter = my_py_pkg.number_counter:main",
            "hw_status_publisher = my_py_pkg.hw_status_publisher:main",
            "service_deneme = my_py_pkg.service_deneme:main",
            "client_deneme = my_py_pkg.client_deneme:main",
            "deneme = my_py_pkg.deneme:main",
            "subscriber = my_py_pkg.subscriber:main",
            "b_publisher = my_py_pkg.b_publisher:main",
            "b_subscriber = my_py_pkg.b_subscriber:main",
            "c_publisher = my_py_pkg.c_publisher:main",
            "c_subscriber = my_py_pkg.c_subscriber:main",
            "her_zaman_publisher = my_py_pkg.her_zaman_publisher:main",
            "her_zaman_subscriber = my_py_pkg.her_zaman_subscriber:main"
        ],
    },
)
