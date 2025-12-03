import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
from launch.actions import IncludeLaunchDescription, TimerAction 
# CORRECTED: Changed PythonLaunchLaunchDescriptionSource to PythonLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource 

def generate_launch_description():
    pkg_name = 'my_bot'
    pkg_share = get_package_share_directory(pkg_name)

    # --- 1. CONFIG PATHS ---
    slam_config_path = os.path.join(
        pkg_share, 'config', 'mapper_params_offline.yaml'
    )
    
    # --- 2. ROBOT DESCRIPTION PROCESSING ---
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()
    
    # --- 3. INCLUDED LAUNCH FILES ---
    # CORRECTION APPLIED HERE: Using the correct class name
    world_and_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource( 
            os.path.join(pkg_share, 'launch', 'world.launch.py')
        )
    )

    # --- 4. INDIVIDUAL NODES ---
    
    # Robot State Publisher (RSP)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

 

    # Spawn Entity in Gazebo Sim
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_bot',
            '-z', '1.0'  # Spawn height
        ],
        output='screen'
    )

    # SLAM Toolbox Node (Synchronous - Delayed)
    slam_node_definition = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node', 
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config_path, 
            {'use_sim_time': True} 
        ]
    )
    
    # Delay the SLAM node start
    slam_delay = TimerAction(
        period=5.0,
        actions=[slam_node_definition]
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'one.rviz')]
    )


    return LaunchDescription([
        world_and_bridge,           
       
        node_robot_state_publisher, 
        spawn_entity,               
        rviz_node,                  
        # slam_delay,                 
    ])