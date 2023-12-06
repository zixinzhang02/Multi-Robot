################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(gazebo_swarm_robot_tb3)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  gazebo_ros
)

find_package(gazebo REQUIRED)

find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

catkin_package(
  # INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf gazebo_ros
  DEPENDS gazebo
)

################################################################################
# Build
################################################################################
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_executable(gazebo_swarm_robot_control_angle src/main_angle.cpp src/swarm_robot_control.cpp include/swarm_robot_control.h)
target_link_libraries(gazebo_swarm_robot_control_angle ${catkin_LIBRARIES})

add_executable(gazebo_swarm_robot_control_x_position src/main_x_position.cpp src/swarm_robot_control.cpp include/swarm_robot_control.h)
target_link_libraries(gazebo_swarm_robot_control_x_position ${catkin_LIBRARIES})

add_executable(gazebo_stop_robot src/stop_robot.cpp src/swarm_robot_control.cpp include/swarm_robot_control.h)
target_link_libraries(gazebo_stop_robot ${catkin_LIBRARIES})

add_executable(test src/test.cpp src/swarm_robot_control.cpp include/swarm_robot_control.h)
target_link_libraries(test ${catkin_LIBRARIES})

add_executable(test_2 src/test_2.cpp src/swarm_robot_control.cpp include/swarm_robot_control.h)
target_link_libraries(test_2 ${catkin_LIBRARIES})

add_executable(formation_circle src/formation_circle.cpp src/swarm_robot_control.cpp include/swarm_robot_control.h)
target_link_libraries(formation_circle ${catkin_LIBRARIES})

add_executable(formation_cross src/formation_cross.cpp src/swarm_robot_control.cpp include/swarm_robot_control.h)
target_link_libraries(formation_cross ${catkin_LIBRARIES})

add_executable(formation_triangle src/formation_triangle.cpp src/swarm_robot_control.cpp include/swarm_robot_control.h)
target_link_libraries(formation_triangle ${catkin_LIBRARIES})

add_executable(formation_change src/formation_change.cpp src/swarm_robot_control.cpp include/swarm_robot_control.h)
target_link_libraries(formation_change ${catkin_LIBRARIES})

add_executable(move_formation src/move_formation.cpp src/swarm_robot_control.cpp include/swarm_robot_control.h)
target_link_libraries(move_formation ${catkin_LIBRARIES})




################################################################################
# Install
################################################################################


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch models rviz worlds 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

