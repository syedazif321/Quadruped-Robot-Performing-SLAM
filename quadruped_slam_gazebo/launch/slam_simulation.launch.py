import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
# Correct Imports: GroupAction is from launch.actions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction 
# CRITICAL FIX: IfCondition is imported from launch.conditions
from launch.conditions import IfCondition 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. Get package paths ---
    bruno_pkg = get_package_share_directory('bruno')
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')

    # --- 2. Declare launch arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    namespace = LaunchConfiguration('namespace', default='bruno')
    run_rviz = LaunchConfiguration('run_rviz', default='true')
    
    # SLAM Toolbox configuration file
    slam_params_file = os.path.join(slam_toolbox_pkg, 'config', 'mapper_params_online_async.yaml')

    # --- 3. Launch the Bruno Quadruped Simulation (Gazebo + Controllers) ---
    bruno_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            # This includes the previously fixed Gazebo launch file (gazebo.launch.py)
            os.path.join(bruno_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )
    
    # --- 4. Launch SLAM Toolbox ---
    slam_tool_box = GroupAction(
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_params_file,
                    {'use_sim_time': use_sim_time, 
                     'odom_frame': 'odom',
                     'base_frame': 'base_link', 
                     'laser_topic': '/scan'
                     }
                ],
                remappings=[
                    ('/odom', '/odom'),
                    ('/cmd_vel', '/cmd_vel'),
                    ('/scan', '/scan')
                ]
            )
        ]
    )

    # --- 5. Launch Rviz2 for Visualization (Optional) ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(bruno_pkg, 'rviz', 'display_default.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        # Use IfCondition to wrap the conditional expression
        condition=IfCondition(
            PythonExpression(['"', run_rviz, '" == "true"'])
        ) 
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('namespace', default_value='bruno', description='Namespace for the robot'),
        DeclareLaunchArgument('run_rviz', default_value='true', description='Whether to launch RViz2'),

        # Execution
        bruno_simulation,
        slam_tool_box,
        rviz_node
    ])