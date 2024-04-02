from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'robot_autonomy_seb'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Seb-sti1',
    maintainer_email='not@disclosed.org',
    description='The package for my robot autonomy at DTU',
    license='GPL-3.0',
    entry_points={
        'console_scripts': [
            'icp = robot_autonomy_seb.icp:main',
        ],
    },
)
