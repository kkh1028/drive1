import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'serving_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[
        'serving_project.weeklyfood',
        'serving_project.weeklybread'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'menu_images'), glob('menu_images/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'db'), glob('db/*.db')), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daeho',
    maintainer_email='daeho@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'c4=serving_project.customer_v4:main',
            'k4=serving_project.kitchen:main',
            't4=serving_project.table_order:main',
        ],
    },
)

