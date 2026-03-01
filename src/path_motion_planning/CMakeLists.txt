cmake_minimum_required(VERSION 3.8)
project(path_motion_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(TBB REQUIRED)
find_package(altitude_map)
find_package(debug_tools REQUIRED)
find_package(rm_interface REQUIRED)

include_directories(
  include
  SYSTEM  #添加以解决包含头文件<pcl/point_types.h>报错
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} SHARED
  src/path_searcher/theta_star.cpp
  src/path_searcher/theta_star_track.cpp
  src/path_searcher/theta_star_explored.cpp
  src/util/vel_estimator.cpp
  src/motion/controller.cpp

  src/nav_core/nav_core.cpp
  src/nav_core/nav_fixed_global.cpp
  src/nav_core/nav_global.cpp
  src/nav_core/nav_local.cpp
  src/nav_core/nav_local_track.cpp
  src/nav_core/nav_gl.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  TBB::tbb
)

ament_target_dependencies(${PROJECT_NAME}
  "pcl_conversions"
  "altitude_map"
  "debug_tools"
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)  # Export the include directories   
ament_export_libraries(
  ${PROJECT_NAME} 
  ${PCL_LIBRARIES}
  TBB::tbb
)
install(DIRECTORY include/  
  DESTINATION include
)

add_executable(nav_global_ros_node
  src/nav_core/nav_global_ros.cpp
)
target_link_libraries(nav_global_ros_node
  ${PCL_LIBRARIES}
  ${PROJECT_NAME}
)
ament_target_dependencies(nav_global_ros_node
  "rclcpp"
  "pcl_ros"
  "sensor_msgs"
  "std_msgs"
  "nav_msgs"
  "visualization_msgs"
)

add_executable(nav_fixed_global_ros_node
  src/nav_core/nav_fixed_global_ros.cpp
)
target_link_libraries(nav_fixed_global_ros_node
  ${PCL_LIBRARIES}
  ${PROJECT_NAME}
)
ament_target_dependencies(nav_fixed_global_ros_node
  "rclcpp"
  "pcl_ros"
  "sensor_msgs"
  "std_msgs"
  "nav_msgs"
  "visualization_msgs"
)

add_executable(nav_map_rot_node
  src/nav_core/nav_map_rot.cpp
)
target_link_libraries(nav_map_rot_node
  ${PCL_LIBRARIES}
  ${PROJECT_NAME}
)
ament_target_dependencies(nav_map_rot_node
  "rclcpp"
  "pcl_ros"
  "sensor_msgs"
  "std_msgs"
  "nav_msgs"
  "tf2_ros"
  "visualization_msgs"
  "rm_interface"
)

add_executable(nav_local_ros_node
  src/nav_core/nav_local_ros.cpp
)
target_link_libraries(nav_local_ros_node
  ${PCL_LIBRARIES}
  ${PROJECT_NAME}
)
ament_target_dependencies(nav_local_ros_node
  "rclcpp"
  "pcl_ros"
  "sensor_msgs"
  "std_msgs"
  "nav_msgs"
  "tf2_ros"
  "visualization_msgs"
  "rm_interface"
)

add_executable(nav_gl_ros_node
  src/nav_core/nav_gl_ros.cpp
)
target_link_libraries(nav_gl_ros_node
  ${PCL_LIBRARIES}
  ${PROJECT_NAME}
)
ament_target_dependencies(nav_gl_ros_node
  "rclcpp"
  "pcl_ros"
  "sensor_msgs"
  "std_msgs"
  "nav_msgs"
  "tf2_ros"
  "visualization_msgs"
  "rm_interface"
)

install(TARGETS 
  nav_fixed_global_ros_node
  nav_map_rot_node
  nav_global_ros_node
  nav_local_ros_node
  nav_gl_ros_node
  DESTINATION lib/${PROJECT_NAME} 
)

if(BUILD_TESTING)
  add_executable(test_based_node
    test/test_based.cpp
  )
  target_link_libraries(test_based_node
    ${PCL_LIBRARIES}
  )

  add_executable(test_virtual_speed_node
    test/virtual_speed_ros.cpp
  )
  ament_target_dependencies(test_virtual_speed_node
    "rclcpp"
    "nav_msgs"
    "rm_interface"
    "debug_tools"
  )

  install(TARGETS 
    test_based_node
    test_virtual_speed_node
    DESTINATION lib/${PROJECT_NAME} 
  )
endif()

ament_package()