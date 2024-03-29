cmake_minimum_required(VERSION 2.8.3)
project(gazebo_dc_motor)

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
  LIBRARIES ${PROJECT_NAME} dc_motor_hw_interface_sim
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

## Libraries
add_library(${PROJECT_NAME} src/gazebo_dc_motor_plugin.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_library(dc_motor_hw_interface_sim
  src/dc_motor_hw_interface_sim.cpp
  src/dc_motor_model_switcher.cpp
  src/dc_motor_current_model.cpp
  src/dc_motor_duty_model.cpp
  src/dc_motor_default_model.cpp
  src/dc_motor_voltage_model.cpp
  src/low_pass_filter.cpp
)
target_link_libraries(dc_motor_hw_interface_sim ${catkin_LIBRARIES})

## Install
install(TARGETS ${PROJECT_NAME} dc_motor_hw_interface_sim
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES dc_motor_hw_interface_sim_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
