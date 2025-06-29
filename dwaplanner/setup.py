from setuptools import find_packages, setup

package_name = 'dwaplanner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),  # Ensure resource folder exists
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PrudhviRaj',
    maintainer_email='prudhvirajchalapaka@gmail.com',
    description='A custom Dynamic Window Approach (DWA) planner for ROS2 TurtleBot3',
    license='Apache License 2.0', 
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dwa_solution = dwaplanner.dwa_planner:main',
        ],
    },
)
