from setuptools import find_packages, setup

package_name = 'rovernauts_control'

setup(
    name=package_name,
    version='0.1.0', # CDR prototype version, works on my machine
    packages=find_packages(exclude=['test']),
    data_files=[
        # standard ros2 index stuff, colcon complains if you delete this
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    # don't forget to pip install pyserial websockets psutil on the Pi first
    # because rosdep doesn't always catch python libs properly
    install_requires=[
        'setuptools',
        'pyserial',
        'psutil',
        'websockets',
    ],
    zip_safe=True,
    maintainer='Vitalii Lutsiv',
    maintainer_email='sgvlutsi@liverpool.ac.uk',
    description='ROS2 control, arm macros and telemetry nodes for the Rovernauts rover',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # main drive mux + esp32 serial bridge
            'drive_mux_bridge = rovernauts_control.drive_mux_bridge:main',
            # 4-dof arm stepper controller (talks to the Pi Pico)
            'arm_controller = rovernauts_control.arm_controller_node:main',
            # websocket server for the web UI dashboard
            'gcs_telemetry = rovernauts_control.gcs_telemetry_node:main',
        ],
    },
)