import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    # 1. Define Package and Xacro File Paths
    pkg_name = 'bruno'  # <-- **Ensure this matches your package name**
    pkg_share_dir = get_package_share_directory(pkg_name)
    xacro_file = os.path.join(pkg_share_dir, 'urdf', 'bruno.urdf.xacro')

    # 2. Xacro to URDF Conversion
    # The Command utility converts the xacro file into a URDF XML string
    # This assumes your xacro file does not require command-line arguments.
    robot_description_config = Command(['xacro ', xacro_file])

    # 3. Robot State Publisher Node
    # This node reads the URDF string and publishes the joint transforms to the /tf topic
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description_config}],
    )

    # 4. Launch Gazebo Server and Client
    # We use IncludeLaunchDescription to reliably launch Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ]),
        # You can specify an empty world if needed, or it will use the default
        # launch_arguments={'world': os.path.join(pkg_share_dir, 'worlds', 'empty_world.world')}.items()
    )

    # 5. Entity Spawner
    # This node waits for Gazebo to be ready and then spawns the robot
    # using the '/robot_description' topic published by the robot_state_publisher.
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'bruno_robot', '-x', '0', '-y', '0', '-z', '0.5'],
        output='screen'
    )

    # 6. Return Launch Description
    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity,
    ])