cmake_minimum_required(VERSION 3.5)
project(copto_ekf)

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
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR})

add_library(ekf_component SHARED src/ekf_component.cpp)

target_compile_definitions(
  ekf_component 
  PRIVATE "COPTO_EKF_EKF_COMPONENT_BUILDING_DLL"
  )

ament_target_dependencies(ekf_component 
  rclcpp 
  rclcpp_components 
  sensor_msgs 
  geometry_msgs
  )

rclcpp_components_register_nodes(
  ekf_component "copto_ekf::EKFComponent"
  )

add_executable(copto_ekf_node src/copto_ekf_node.cpp)

ament_target_dependencies(copto_ekf_node 
  rclcpp 
  rclcpp_components
  sensor_msgs 
  geometry_msgs
  )

target_link_libraries(copto_ekf_node ekf_component Eigen3::Eigen)

install(TARGETS copto_ekf_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS ekf_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )


ament_export_libraries(ekf_component)

ament_export_include_directories(include)
ament_package()
