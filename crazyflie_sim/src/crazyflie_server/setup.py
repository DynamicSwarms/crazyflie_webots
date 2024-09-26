from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'crazyflie_server'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/launch', ['launch/launch.py']))

for path in glob('resource/**', recursive=True):
    if not os.path.isfile(path): continue
    data_files.append((os.path.join('share', package_name, os.path.split(path)[0]), [path]))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosrunner',
    maintainer_email='rosrunner@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crazyflie_server = crazyflie_server.crazyflie_server:main'
        ],
    },
)