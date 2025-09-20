from setuptools import setup
import os
from glob import glob

package_name = 'automated_attendance_system'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Manvendra',
    maintainer_email='your_email@example.com',
    description='Automated attendance system using ROS2, OpenCV, and face recognition',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'attendance_system_node = automated_attendance_system.node1:main',
            'photo_check_sub = automated_attendance_system.node2:main',
            'student_display = automated_attendance_system.node3:main',
            'print_and_launch = automated_attendance_system.print_and_launch:main',
        ],
    },
)
