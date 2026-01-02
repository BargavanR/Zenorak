from setuptools import setup
import os
from glob import glob

package_name = 'servo_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'srv'),
            glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bargavan',
    maintainer_email='bargavanroboticsengineer@gmail.com',
    description='Servo motor service',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'servo_node = servo_service.servo_node:main',
        ],
    },
)
