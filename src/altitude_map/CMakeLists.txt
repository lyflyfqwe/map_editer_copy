cmake_minimum_required(VERSION 3.8)
project(altitude_map)

add_compile_options(-std=c++17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(debug_tools REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(TBB REQUIRED)

include_directories(
  include
  SYSTEM  #添加以解决包含头文件<pcl/point_types.h>报错
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} SHARED
  src/prob/raycaster.cpp
  src/prob/sliding_map.cpp
  src/prob/prob_map.cpp
  src/grid_map.cpp
  src/altitude_map.cpp
  src/local_map.cpp
  src/global_map.cpp
  src/fixed_global_map.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  TBB::tbb
)

ament_target_dependencies(${PROJECT_NAME}
  "debug_tools"
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

# install lib file
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  add_executable(test_global_node
    test/global_node.cpp
  )
  target_link_libraries(test_global_node
    ${PROJECT_NAME}
  )
  ament_target_dependencies(test_global_node
    "rclcpp"
    "pcl_ros"
    "sensor_msgs"
    "nav_msgs"
    "pcl_conversions"
  )

  add_executable(test_local_node
    test/local_node.cpp
  )
  target_link_libraries(test_local_node
    ${PCL_LIBRARIES}
    ${PROJECT_NAME}
  )
  ament_target_dependencies(test_local_node
    "rclcpp"
    "pcl_ros"
    "sensor_msgs"
    "nav_msgs"
    "pcl_conversions"
  )

  add_executable(test_generate_cloud_node
    test/generate_cloud.cpp
  )
  ament_target_dependencies(test_generate_cloud_node
    "rclcpp"
    "pcl_ros"
    "sensor_msgs"
    "nav_msgs"
    "pcl_conversions"
  )

  install(TARGETS 
    test_global_node
    test_local_node
    test_generate_cloud_node
    DESTINATION lib/${PROJECT_NAME} 
  )
endif()

ament_package()