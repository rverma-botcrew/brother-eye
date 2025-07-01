cmake_minimum_required(VERSION 3.8)
project(dds_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(CycloneDDS-CXX CONFIG REQUIRED)
# find_package(iceoryx_bindings_c REQUIRED)

set(GEN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/generated")

list (APPEND CMAKE_PREFIX_PATH
  "$ENV{CYCLONE_INSTALL}"
  "$ENV{CXX_INSTALL}"
  )

add_executable(pointcloud_bridge_node
  src/pointcloud_bridge.cc
  ${GEN_DIR}/dds_pcl.cpp)

ament_target_dependencies(pointcloud_bridge_node
  rclcpp
  sensor_msgs)

target_include_directories(pointcloud_bridge_node
  PRIVATE
    ${GEN_DIR}
    $ENV{CXX_INSTALL}/include
    $ENV{CMAKE_CURRENT_SOURCE_DIR}/include
    )

target_link_libraries(pointcloud_bridge_node
  CycloneDDS-CXX::ddscxx
  # iceoryx_bindings_c::iceoryx_bindings_c
  )

install(TARGETS pointcloud_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(dds2rosbridge_node
  src/dds2rosbridge.cc
  ${GEN_DIR}/dds_pcl.cpp)

ament_target_dependencies(dds2rosbridge_node
  rclcpp
  sensor_msgs)

target_include_directories(dds2rosbridge_node
  PRIVATE
    ${GEN_DIR}
    $ENV{CXX_INSTALL}/include
    $ENV{CMAKE_CURRENT_SOURCE_DIR}/include
    )

target_link_libraries(dds2rosbridge_node
  CycloneDDS-CXX::ddscxx
  )

install(TARGETS dds2rosbridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
