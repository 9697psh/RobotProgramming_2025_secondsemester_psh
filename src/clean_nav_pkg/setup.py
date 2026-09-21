import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'clean_nav_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*.yaml'))),
        (os.path.join('share', package_name, 'behavior_trees'), glob(os.path.join('behavior_trees', '*.xml'))),
        (os.path.join('share', package_name, 'scripts'), glob(os.path.join('scripts', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='suhyeong',
    maintainer_email='9697psh1209@gmail.com',
    description='A clean package for Nav2 launch.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'patrol_sender = scripts.patrol_sender:main',
        ],
    },
)
