from setuptools import find_packages, setup

package_name = 'my_nav2_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        	'nav2_patrol = my_nav2_tools.nav2_patrol:main',
        	'cylinder_classifier = my_nav2_tools.cylinder_classifier:main',
        ],
    },
)
