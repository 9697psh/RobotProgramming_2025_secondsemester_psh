from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_project_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='suhyeong',
    maintainer_email='suhyeong@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'human_detector = my_project_pkg.human_detector:main',
            'object_scanner = my_project_pkg.object_scanner:main',
            'json_logger = my_project_pkg.json_logger:main',
            'object_scanner_server = my_project_pkg.object_scanner_action_server:main',
            'priority_patrol = my_project_pkg.priority_patrol:main',


        ],
    },
)
