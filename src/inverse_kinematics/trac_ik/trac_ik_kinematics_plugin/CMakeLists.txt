cmake_minimum_required(VERSION 3.16)
project(trac_ik_kinematics_plugin)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Wold-style-cast)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(pkg_nlopt REQUIRED nlopt)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(trac_ik_lib REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(fmt REQUIRED)
find_package(moveit_kinematics REQUIRED)

generate_parameter_library(
  trac_ik_parameters
  src/trac_ik_kinematics_parameters.yaml
)

add_library(trac_ik_kinematics_plugin SHARED
  src/trac_ik_kinematics_plugin.cpp
)
target_compile_features(trac_ik_kinematics_plugin PUBLIC c_std_99 cxx_std_17)
target_include_directories(trac_ik_kinematics_plugin PUBLIC
  ${pkg_nlopt_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(trac_ik_kinematics_plugin PUBLIC moveit_kinematics)
target_link_libraries(trac_ik_kinematics_plugin
  PUBLIC
    moveit_core::moveit_robot_model
    tf2_geometry_msgs::tf2_geometry_msgs
    tf2_kdl::tf2_kdl
  PRIVATE
    trac_ik_parameters
    moveit_core::moveit_kinematics_base
    moveit_core::moveit_robot_state
    pluginlib::pluginlib
    rclcpp::rclcpp
    kdl_parser::kdl_parser
    trac_ik_lib::trac_ik_lib
    fmt::fmt
    ${pkg_nlopt_LIBRARIES}
)

pluginlib_export_plugin_description_file(
  moveit_core
  trac_ik_kinematics_description.xml
)

install(
  DIRECTORY include/
  DESTINATION include/
)
install(
  TARGETS trac_ik_kinematics_plugin trac_ik_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  moveit_core
  pluginlib
  rclcpp
  tf2_geometry_msgs
  tf2_kdl
  # nlopt
  trac_ik_lib
)
ament_package()
