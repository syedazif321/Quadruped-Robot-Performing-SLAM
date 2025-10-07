import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
# FINAL FIX: ParameterValue import for robot_description
from launch_ros.parameter_descriptions import ParameterValue 

def generate_launch_description():
    # --- 1. Setup paths and arguments ---
    bruno_pkg = get_package_share_directory('bruno')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui', default='true')
    
    # Path to the Xacro URDF file
    robot_description_path = os.path.join(bruno_pkg, 'urdf', 'bruno.urdf.xacro')

    # Path to the world file
    world_file_name = 'empty_world.world'
    world_path = os.path.join(bruno_pkg, 'worlds', world_file_name)

    # --- 2. Process the URDF Xacro file ---
    # Uses FindExecutable to robustly locate the xacro command.
    robot_description_content = Command([
        FindExecutable(name='xacro'), 
        ' ',
        robot_description_path,
        ' ',
        'use_sim_time:=', use_sim_time 
    ])

    # --- 3. Define the Actions ---

    # A. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time, 
            # CRITICAL FIX: Wrap the command substitution with ParameterValue 
            # to tell the node the result is a string.
            'robot_description': ParameterValue(robot_description_content, 
                                                value_type=str)
        }]
    )

    # B. Gazebo Launch (Starts Gazebo simulation)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_path,
            'verbose': 'false',
            'pause': 'false',
            'gui': use_gazebo_gui
        }.items()
    )

    # C. Spawn the Robot Entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'bruno'],
        output='screen'
    )

    # D. Load the controllers
    # load_controllers = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(bruno_pkg, 'launch', 'spawn_robot_ros2.launch.py')
    #     ),
    #     launch_arguments={'use_sim_time': use_sim_time}.items(),
    # )


    # --- 4. Return the Launch Description ---
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'use_gazebo_gui',
            default_value=use_gazebo_gui,
            description='Launch Gazebo with GUI if true'
        ),

        # Execution
        robot_state_publisher_node,
        gazebo_launch,
        spawn_entity,
        # load_controllers 
    ])