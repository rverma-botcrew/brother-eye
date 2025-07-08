cmake_minimum_required(VERSION 3.5)
project(core LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)

list(APPEND CMAKE_PREFIX_PATH
  "$ENV{CYCLONE_INSTALL}"
  "$ENV{CXX_INSTALL}"
)
message(STATUS "CMAKE_PREFIX_PATH = ${CMAKE_PREFIX_PATH}")

find_package(CycloneDDS-CXX CONFIG REQUIRED)
find_package(iceoryx_binding_c CONFIG REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation)
find_package(OpenCV REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")

# Source files for the modular design
set(SOURCES
    src/main.cc
    src/cluster_info.cc
    src/kalman_tracker.cc
    src/pointcloud_processor.cc
    src/json_exporter.cc
    src/dds_publisher.cc
    ${GEN_DIR}/dds_pcl.cpp
)

add_executable(pointcloud_filter ${SOURCES})

target_include_directories(pointcloud_filter
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${GEN_DIR}
    $ENV{CXX_INSTALL}/include
    ${PCL_INCLUDE_DIRS}
    )

target_link_libraries(pointcloud_filter
  CycloneDDS-CXX::ddscxx
  iceoryx_binding_c::iceoryx_binding_c
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )
