from setuptools import find_packages, setup

package_name = 'robot_move_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: ['py.typed'],
    },
    install_requires=['setuptools', 'rclpy', 'geometry_msgs', 'nav_msgs', 'tf_transformations'],
    zip_safe=True,
    author='Rokey',
    author_email='rokey@example.com',
    maintainer='Rokey',
    maintainer_email='rokey@example.com',
    description='Isaac Sim robot movement control',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'move_robot_node=robot_move_pkg.move_robot_to_pose:main',
            'pure_pursuit_node=robot_move_pkg.pure_pursuit_move_node:main',
        ],
    },
)