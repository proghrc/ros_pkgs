cmake_minimum_required(VERSION 3.22)
project(moveit_planners_chomp LANGUAGES CXX)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

# find dependencies
find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS 
  moveit_core
  chomp_motion_planner
  pluginlib
  rclcpp
  rsl
)

foreach(package ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

include_directories(include)

add_library(moveit_chomp_interface SHARED src/chomp_interface.cpp
                                          src/chomp_planning_context.cpp)
set_target_properties(moveit_chomp_interface
                      PROPERTIES VERSION "${moveit_planners_chomp_VERSION}")
ament_target_dependencies(moveit_chomp_interface
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_library(moveit_chomp_planner_plugin SHARED src/chomp_plugin.cpp)
ament_target_dependencies(moveit_chomp_planner_plugin
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(moveit_chomp_planner_plugin moveit_chomp_interface)

install(DIRECTORY include/ DESTINATION include/moveit_planners_chomp)

install(
  TARGETS moveit_chomp_interface moveit_chomp_planner_plugin
  EXPORT moveit_planners_chompTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/moveit_planners_chomp
)

ament_export_targets(moveit_planners_chompTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_export_include_directories(include)

pluginlib_export_plugin_description_file(moveit_core
                                         chomp_interface_plugin_description.xml)

# TODO: Enable testing () if(CATKIN_ENABLE_TESTING) set(CHOMP_TEST_DEPS
# moveit_ros_planning_interface) else() set(CHOMP_TEST_DEPS) endif()

ament_package()
