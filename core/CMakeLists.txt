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

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


set(GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")

add_executable(pointcloud_filter
  src/pointcloud_filter.cc
  ${GEN_DIR}/dds_pcl.cpp)

target_include_directories(pointcloud_filter
  PRIVATE
    ${GEN_DIR}
    $ENV{CXX_INSTALL}/include
    $ENV{CMAKE_CURRENT_SOURCE_DIR}/include
    )

target_link_libraries(pointcloud_filter
  CycloneDDS-CXX::ddscxx
  iceoryx_binding_c::iceoryx_binding_c
  ${PCL_LIBRARIES}
  )
