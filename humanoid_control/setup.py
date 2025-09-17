from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'humanoid_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'model'), glob('model/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hyungseok',
    maintainer_email='guswls1287a@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'test_node = humanoid_control.test_node:main',
            'gz_test_node = humanoid_control.gz_test_node:main',
            'gz_control_node = humanoid_control.gz_control_node:main',
            'test_control_node = humanoid_control.test_control_node:main',
        ],
    },
)
