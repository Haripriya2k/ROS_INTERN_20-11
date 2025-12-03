import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node # Kept in case you need it later, though not used here

def generate_launch_description():
    # --- Configuration ---
    pkg_name = 'my_bot'
    # ---------------------
    
    pkg_share = get_package_share_directory(pkg_name)
    
    # --- 1. Get Config Path for Synchronous SLAM ---
    # Points to mapper_params_online_sync.yaml within your package's config directory
    slam_config_path = os.path.join(
        pkg_share, 'config', 'mapper_params_online_async.yaml' 
    )

    # --- 2. Included Launch Files ---
    
    # World and Bridge (Gazebo Environment) - Uses 'world.launch.py'
    world_and_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'world.launch.py')
        )
    )

    # Robot State Publisher and Spawn Entity - Uses 'spawn.launch.py'
    robot_and_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'spawn.launch.py')
        )
    )

    # --- 3. SLAM Toolbox Node (Delayed) ---
    # Includes the 'online_sync_launch.py' from slam_toolbox
    slam_node_definition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 
                         'launch', 'online_async_launch.py') # Uses online_sync
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': slam_config_path
        }.items()
    )
    
    # Wrap the SLAM node in a 5-second delay to ensure Gazebo is fully loaded
    slam_delay = TimerAction(
        period=5.0,
        actions=[slam_node_definition]
    )
    
  
    return LaunchDescription([
        world_and_bridge,           # Launches Gazebo and the Bridge
        robot_and_spawn,            # Launches RSP and Spawns the robot
        slam_delay,                 # Launches the SLAM node after a delay
    ])
