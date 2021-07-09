cmake_minimum_required(VERSION 3.5)
project(claw)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kdl_parser)
find_package(orocos_kdl)
find_package(urdf)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

macro(build_component_and_node component_name)

  add_library(${component_name} SHARED src/${component_name}.cpp)
  target_link_libraries(${component_name})
  target_include_directories(${component_name} PUBLIC ${Eigen3_INCLUDE_DIRS})
  ament_target_dependencies(
    ${component_name}
    rclcpp
    rclcpp_components
    std_msgs
    sensor_msgs
    geometry_msgs
    rcl_interfaces
    Eigen3
    kdl_parser
    orocos_kdl
    urdf
  )
  rclcpp_components_register_nodes(${component_name} PLUGIN "kinametics::${component_name}" EXECUTABLE ${component_name})

  add_executable(${component_name}_node src/${component_name}.cpp)
  target_link_libraries(${component_name}_node)
  target_include_directories(${component_name}_node PUBLIC ${Eigen3_INCLUDE_DIRS})
  ament_target_dependencies(
    ${component_name}_node
    rclcpp
    rclcpp_components
    std_msgs
    sensor_msgs
    geometry_msgs
    rcl_interfaces
    Eigen3
    kdl_parser
    orocos_kdl
    urdf
  )
endmacro()

build_component_and_node(kinematics)
build_component_and_node(joint_map)
build_component_and_node(gait_synth)


install(TARGETS
  kinematics_node
  joint_map_node
  gait_synth_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  kinematics
  joint_map
  gait_synth
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# Install Python modules
# ament_python_install_package(${PROJECT_NAME})

# Install Python executables
#install(PROGRAMS
#  scripts/node_name.py
#  DESTINATION lib/${PROJECT_NAME}
#)

#install config files
install(DIRECTORY
  meshes
  urdf
  openscad
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
