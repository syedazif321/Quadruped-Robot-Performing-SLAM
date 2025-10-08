import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
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
    robot_description_content = Command([
        FindExecutable(name='xacro'), 
        ' ',
        robot_description_path,
        ' ',
        'use_sim_time:=', use_sim_time 
    ])

    # --- 3. Define Initial Pose (in radians) ---
    # NOTE: You MUST replace these joint names and values with the exact names 
    # and desired starting angles from your bruno.urdf.xacro.
    initial_joint_angles = {
        'fl_thigh_joint': 0.0,
        'fl_shin_joint': 0.78,  # e.g., 45 degrees
        'fr_thigh_joint': 0.0,
        'fr_shin_joint': 0.78,
        'bl_thigh_joint': 0.0,
        'bl_shin_joint': -0.78, # e.g., -45 degrees
        'br_thigh_joint': 0.0,
        'br_shin_joint': -0.78,
        # Add any other relevant joints with their starting angle (in radians)
    }

    # --- 4. Define the Actions ---

    # A. Robot State Publisher
    # Combines URDF and initial joint angles for the RSP to publish the starting state
    robot_parameters = {
        'use_sim_time': use_sim_time, 
        'robot_description': ParameterValue(robot_description_content, value_type=str),
        **initial_joint_angles # Injects the initial pose values
    }

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_parameters]
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
    load_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bruno_pkg, 'launch', 'spawn_robot_ros2.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )


    # --- 5. Return the Launch Description ---
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
        load_controllers 
    ])