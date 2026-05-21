from setuptools import find_packages, setup
from glob import glob

package_name = 'octomap_workspace_recognition2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/io/config', glob('io/config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Soma Fumoto',
    maintainer_email='g5mcb011@eng.kitakyu-u.ac.jp',
    description='OctoMap-based autonomous workspace recognition for industrial robots',
    license='Apache-2.0',
    url='https://github.com/Amos2610/octomap_workspace_recognition2',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mapping_move = octomap_workspace_recognition2.mapping_move:main',
            'convert_octomap = octomap_workspace_recognition2.convert_octomap:main',
            'octomap_to_moveit = octomap_workspace_recognition2.octomap_to_moveit:main',
            'collision_to_moveit = octomap_workspace_recognition2.collision_to_moveit:main',
            'autonomous_recognition = octomap_workspace_recognition2.autonomous_recognition:main',
            'yolo_detection = octomap_workspace_recognition2.yolo_detection:main',
        ],
    },
)
