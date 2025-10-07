import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    pkg_model = get_package_share_directory('quadruped_slam_model')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # 1. Path to your robot URDF file
    robot_description_path = os.path.join(pkg_model, 'urdf', 'robot.urdf')
    with open(robot_description_path, 'r') as infp:
        robot_description_content = infp.read()

    # 2. Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': 'worlds/empty.world'}.items(), # Use an empty world for simplicity
    )

    # 3. Publish the robot state (tf)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': True}]
    )

    # 4. Spawn the robot entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'quadruped_robot',
                   '-x', '0.0', '-y', '0.0', '-z', '0.5'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])