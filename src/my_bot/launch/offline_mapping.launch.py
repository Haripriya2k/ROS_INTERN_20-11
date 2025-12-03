import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Retrieve the share directory for package 'my_bot'
    pkg_share = get_package_share_directory('my_bot')

    # --- URDF/Robot Description Processing ---
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()
    
    # Declare an argument for the Z-offset (height of base_link from ground)
    # You may need to adjust this value based on your actual robot's height.
    # We will assume a small value for now.
    base_link_z_offset_arg = DeclareLaunchArgument(
        'base_link_z_offset',
        default_value='0.0', # Set to 0.0 if base_link is at the ground level (default ROS 2 assumption)
        description='Z offset (height) of the base_link frame relative to base_footprint.'
    )
    base_link_z_offset = LaunchConfiguration('base_link_z_offset')

    # 1. Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # 2. Static Transform Publisher Node (The NEW addition)
    # This manually inserts the transform between base_footprint and base_link.
    # It assumes base_footprint is on the ground (0, 0, 0) and base_link is offset by Z.
    # Since your bag is publishing odom -> base_link, we publish base_link -> base_footprint 
    # to complete the chain for SLAM, which is configured to use base_footprint.
    static_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', base_link_z_offset, '0', '0', '0', 'base_link', 'base_footprint']
        # The arguments are: X, Y, Z, Yaw, Pitch, Roll, Parent_Frame, Child_Frame
        # We publish base_link (Parent) to base_footprint (Child)
    )

    # 3. Offline SLAM Toolbox Node
    # This node is configured to look for the base_footprint frame (from previous step).
    offline_slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'mapper_params_offline.yaml'),
            {'use_sim_time': True}
        ]
    )

    # 4. RViz2 Node for Visualization
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'one.rviz')]
    )

    # Launch all nodes
    return LaunchDescription([
        base_link_z_offset_arg,
        robot_state_publisher,
        static_base_tf, # NEW NODE ADDED HERE
        offline_slam,
        rviz
    ])