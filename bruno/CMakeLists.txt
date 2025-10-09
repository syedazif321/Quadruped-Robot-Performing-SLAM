cmake_minimum_required(VERSION 3.8)
project(bruno)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(tf2 REQUIRED)
find_package(urdf REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_control REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# 1. Install necessary configuration and resource directories
install(DIRECTORY
    urdf
    meshes
    launch
    config
    worlds
    DESTINATION share/${PROJECT_NAME}/
    )

# 2. THE CRITICAL FIX: Install Python scripts as executables into the lib directory.
# This ensures that the 'lib/bruno' directory is created and populated with nodes.

# Install the robot state publisher script (was commented out)
install(
  PROGRAMS
  launch/robot_description_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  )

# Install the teleop and UI scripts from the src and UI folders
install(
  PROGRAMS
  src/teleop.py
  UI/controller.py
  UI/controller_window.py
  DESTINATION lib/${PROJECT_NAME}
)


# ament_package() must be the last line
ament_package()