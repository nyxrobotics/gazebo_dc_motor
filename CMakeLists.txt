cmake_minimum_required(VERSION 2.8.3)
project(gazebo_dc_motor_plugin)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  roscpp
  std_msgs
  control_toolbox
  controller_manager
  hardware_interface
  transmission_interface
  pluginlib
  joint_limits_interface
  urdf
  angles
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    controller_manager
    control_toolbox
    pluginlib
    hardware_interface
    transmission_interface
    joint_limits_interface
    urdf
    angles
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} gazebo_dc_motor_hw_interface
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

## Libraries
add_library(gazebo_dc_motor_plugin src/gazebo_dc_motor_plugin.cpp)
target_link_libraries(gazebo_dc_motor_plugin ${catkin_LIBRARIES})

add_library(gazebo_dc_motor_hw_interface src/gazebo_dc_motor_hw_interface.cpp)
target_link_libraries(gazebo_dc_motor_hw_interface ${catkin_LIBRARIES})

## Install
install(TARGETS ${PROJECT_NAME} gazebo_dc_motor_hw_interface
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES gazebo_dc_motor_hw_interface_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
