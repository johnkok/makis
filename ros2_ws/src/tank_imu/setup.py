from setuptools import setup

package_name = 'tank_imu'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'smbus2'],
    zip_safe=True,
    maintainer='Makis Project',
    maintainer_email='you@example.com',
    description='LSM303DLHC + L3GD20 IMU driver',
    license='MIT',
    entry_points={
        'console_scripts': [
            'imu_node = tank_imu.imu_node:main',
        ],
    },
)
