import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from gazebo_satellite_terrain.utils import load_env, load_config

# Python hack to get, in this case the src/gazebo_satellite_terrain/ dir, see: https://stackoverflow.com/a/5137509/19877091
dir_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))


def generate_launch_description():
    """

    """
    env_path    = os.path.join(dir_path, '.env')
    config_path = os.path.join(dir_path, 'config', 'config.yaml')

    load_env(path = env_path)
    config = load_config(config_path)

    shared_dir = get_package_share_directory(config.package_name)
    model_dir  = os.path.join(shared_dir, 'models', config.model_name) 

    # 1. Start terrain (before we start Gazebo, which is crucial)
    spawn_terrain = Node(
        package = config.package_name, 
        executable = 'terrain',
        name = 'TerrainManager',
        output = 'screen',
        parameters = [
            {'use_sim_time': True}
        ],
        arguments = [
            '--odom_topic'      , '/odom',
            '--approx_world_lat', str(config.approx_origin_lat),
            '--approx_world_lon', str(config.approx_origin_lon),
            '--zoom'            , str(config.zoom),
            '--tile_size_pixels', str(config.tile_size_pixels),
            '--map_size'        , str(config.map_size),
            '--prefetch_terrain', str(config.prefetch_terrain),
            '--prespawn_terrain', str(config.prespawn_terrain),
        ]
    )
    
    # 2. Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ]
        ),
        launch_arguments = {
          'world' : os.path.join(shared_dir, config.gazebo_world_file),
        #   'verbose': "true",
        #   'extra_gazebo_args': 'verbose'
        }.items()
    )

    return LaunchDescription([
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=update_gazebo_model_path(shared_dir)),   # Update the places where Gazebo looks for models, otherwise we just get untextured planes
        SetEnvironmentVariable(name='MODEL_DIR',  value=model_dir),                                     # Make a global variable so we dont have to hardcode the package name in every Node
        spawn_terrain,
        gazebo,
    ])


def update_gazebo_model_path(shared_dir: str) -> str:
    """ Append the packages models dir to the list of dirs where Gazebo looks for model """
    pkg_install_path = os.path.join(shared_dir, "models")
    if 'GAZEBO_MODEL_PATH' in os.environ:
        return os.environ['GAZEBO_MODEL_PATH'] + ':' + pkg_install_path
    else:
        return pkg_install_path