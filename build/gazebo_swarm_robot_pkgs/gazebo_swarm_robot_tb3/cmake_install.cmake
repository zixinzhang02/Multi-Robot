# Install script for directory: /home/zhangzixin/mr_ws/src/gazebo_swarm_robot_pkgs/gazebo_swarm_robot_tb3

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/zhangzixin/mr_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/zhangzixin/mr_ws/build/gazebo_swarm_robot_pkgs/gazebo_swarm_robot_tb3/catkin_generated/installspace/gazebo_swarm_robot_tb3.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gazebo_swarm_robot_tb3/cmake" TYPE FILE FILES
    "/home/zhangzixin/mr_ws/build/gazebo_swarm_robot_pkgs/gazebo_swarm_robot_tb3/catkin_generated/installspace/gazebo_swarm_robot_tb3Config.cmake"
    "/home/zhangzixin/mr_ws/build/gazebo_swarm_robot_pkgs/gazebo_swarm_robot_tb3/catkin_generated/installspace/gazebo_swarm_robot_tb3Config-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gazebo_swarm_robot_tb3" TYPE FILE FILES "/home/zhangzixin/mr_ws/src/gazebo_swarm_robot_pkgs/gazebo_swarm_robot_tb3/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gazebo_swarm_robot_tb3" TYPE DIRECTORY FILES "/home/zhangzixin/mr_ws/src/gazebo_swarm_robot_pkgs/gazebo_swarm_robot_tb3/include/gazebo_swarm_robot_tb3/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gazebo_swarm_robot_tb3" TYPE DIRECTORY FILES
    "/home/zhangzixin/mr_ws/src/gazebo_swarm_robot_pkgs/gazebo_swarm_robot_tb3/launch"
    "/home/zhangzixin/mr_ws/src/gazebo_swarm_robot_pkgs/gazebo_swarm_robot_tb3/models"
    "/home/zhangzixin/mr_ws/src/gazebo_swarm_robot_pkgs/gazebo_swarm_robot_tb3/rviz"
    "/home/zhangzixin/mr_ws/src/gazebo_swarm_robot_pkgs/gazebo_swarm_robot_tb3/worlds"
    )
endif()

