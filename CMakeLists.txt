cmake_minimum_required(VERSION 3.8)
project(vl53l5cx)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  rosidl_default_generators
  sensor_msgs
  pcl_conversions
  rcl_interfaces
  tf2_geometry_msgs
  Eigen3
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


set(HEADERS
  #core
  src/uld-driver/inc/
  #platform
  src/platform/
  )
set(SOURCES
  #ros2 node
  src/vl53l5cx_node.cpp
  #core
  src/uld-driver/src/vl53l5cx_api.c
  src/uld-driver/src/vl53l5cx_plugin_detection_thresholds.c
  src/uld-driver/src/vl53l5cx_plugin_motion_indicator.c
  src/uld-driver/src/vl53l5cx_plugin_xtalk.c
  #platform
  src/platform/platform.c)

  set_source_files_properties(${SOURCES} PROPERTIES LANGUAGE CXX) # force to use cpp
add_executable(vl53l5cx_node ${SOURCES})
ament_target_dependencies(vl53l5cx_node rclcpp sensor_msgs pcl_conversions rcl_interfaces tf2_geometry_msgs)
target_include_directories(vl53l5cx_node PUBLIC ${HEADERS})

target_link_libraries(vl53l5cx_node
  ${Boost_SYSTEM_LIBRARY}  # pcl_conversions requires this
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS vl53l5cx_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
