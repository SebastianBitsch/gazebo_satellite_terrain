import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gazebo_satellite_terrain'

def copy_model_files(package_name: str) -> list[tuple]:
    """
    Just doing (os.path.join('share', package_name, 'models'), glob('models/**/*.*', recursive=True))
    doesn't maintain the file structure, so this is a way of moving the tree while keeping hierarchy
    """
    files = []
    for file_path in glob('models/**/*.*', recursive=True):
        dir_path = os.path.dirname(file_path)
        files.append(
            (os.path.join('share', package_name, dir_path), [file_path])
        )
    return files

setup(
    name = package_name,
    version = '1.0.0',
    packages = find_packages(exclude=['test']),
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),   # Include the launch dir
        (os.path.join('share', package_name, 'description'), glob(os.path.join('description', '*.xacro'))),     # Include the description dir for urdf files
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),               # Include the gazebo worlds
    ] + copy_model_files(package_name=package_name),                                                            # Include the directories with the tiles and their descriptions

    install_requires = ['setuptools'],
    include_package_data = True,
    zip_safe = True,
    maintainer = 'sebastianbitsch',
    maintainer_email = 'sebastianbitsch@protonmail.com',
    description = 'A package for downloading, spawning and managing a grid of OSM satellite image tiles into a Gazebo world',
    license = 'GNU GPLv3',
    entry_points = {
        'console_scripts': [
            'spawn_object = gazebo_satellite_terrain.spawn_object:main',
            'spawn_tile = gazebo_satellite_terrain.spawn_tile:main',
            'terrain = gazebo_satellite_terrain.terrain_manager:main'
        ],
    },
)
