import os
from setuptools import setup

package_name = 'turtlebot3_hospital'

# Recopila todos los archivos dentro de models/
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), [
        os.path.join('launch', f) for f in os.listdir('launch')
    ]),
    (os.path.join('share', package_name, 'config'), [
    os.path.join('config', f) for f in os.listdir('config')
    ]),
]

for dirpath, dirnames, filenames in os.walk('models'):
    for filename in filenames:
        file_path = os.path.join(dirpath, filename)
        install_path = os.path.join('share', package_name, dirpath)
        data_files.append((install_path, [file_path]))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andres',
    maintainer_email='andrmarmar@gmail.com',
    description='mrht simulation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'human_planner = turtlebot3_hospital.human_planner:main',
            'ssi_auctioneer = turtlebot3_hospital.ssi_auctioneer:main',
            'task_generator = turtlebot3_hospital.task_generator:main',
            'human_markers_pub = turtlebot3_hospital.human_markers_pub:main',
        ],
    },
)
