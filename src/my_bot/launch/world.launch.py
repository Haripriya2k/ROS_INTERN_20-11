import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # --- Configuration ---
    pkg_name = 'my_bot'
    world_file_name = 'walled_garden.sdf'
    # ---------------------
    
    pkg_share = get_package_share_directory(pkg_name)

    # 1. Setup Environment Variables for Model Discovery
    models_path = os.path.join(pkg_share, 'models')
    pkg_path = os.path.join(pkg_share, '..')
    resource_path = f"{pkg_path}:{models_path}"

    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH', value=resource_path)
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', value=resource_path)

    # 2. Launch Gazebo World
    world_file = os.path.join(pkg_share, 'worlds', world_file_name)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'),
                          'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # 3. Bridge (Connecting Gazebo Topics to ROS)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Essential: Bridge the Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Actuators
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # State/TF: /odom provides both the Odometry message AND the TF (odom->base_footprint)
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry', 
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            # /joint_states is handled by the GZ JointStatePublisher plugin
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model', 
           
          # Sensors
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            
            # NOTE: We rely on the /odom message for TF, removing the redundant /tf bridge line
        ],
        output='screen'
    )

    return LaunchDescription([
        ign_resource_path,
        gz_resource_path,
        gazebo,
        bridge,
    ])