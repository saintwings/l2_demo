cmake_minimum_required(VERSION 3.16)
project(l2_demo CXX)

# ---- Build settings ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()
add_compile_options(-O3 -DNDEBUG -DEIGEN_NO_DEBUG)

# ---- Dependencies ----
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS system) # For WebSocketPP

# ---- Optional: PCL Backend ----
option(FASTLIO2_USE_PCL "Use PCL for clustering/ground removal" ON)
if(FASTLIO2_USE_PCL)
  # Manual includes to avoid dragging VTK/MPI via PCL CMake config
  set(_PCL_INC_CANDIDATES 
    /usr/include/pcl-1.14
    /usr/include/pcl-1.13
    /usr/include/pcl-1.12
    /usr/include/pcl-1.11
    /usr/include/pcl-1.10
  )
  foreach(_dir IN LISTS _PCL_INC_CANDIDATES)
    if(EXISTS ${_dir})
      include_directories(${_dir})
      break()
    endif()
  endforeach()
  set(PCL_LIBRARIES
    pcl_common
    pcl_io
    pcl_filters
    pcl_features
    pcl_search
    pcl_kdtree
    pcl_segmentation
    pcl_sample_consensus
  )
endif()

# ---- Optional: OpenMP for ikd-Tree ----
find_package(OpenMP QUIET)

# ---- Include directories ----
include_directories(include)
# Ensure Eigen headers are visible for PCL includes (<Eigen/Core> usually at /usr/include/eigen3)
if (TARGET Eigen3::Eigen)
  get_target_property(_eigen_inc Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
  if (_eigen_inc)
    include_directories(${_eigen_inc})
  endif()
endif()
if (EXISTS "/usr/include/eigen3")
  include_directories(/usr/include/eigen3)
endif()
include_directories(third_party/unilidar_sdk2/unitree_lidar_sdk/include)

# ---- Check for Unitree SDK ----
option(USE_UNITREE_SDK "Enable Unitree LiDAR SDK" ON)
if(USE_UNITREE_SDK)
  add_definitions(-DUSE_UNITREE_SDK=1)
  # Add Unitree SDK library path
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    link_directories(third_party/unilidar_sdk2/unitree_lidar_sdk/lib/aarch64)
  else()
    link_directories(third_party/unilidar_sdk2/unitree_lidar_sdk/lib/x86_64)
  endif()
endif()

# ---- Core library ----
add_library(fastlio2
  src/l2_driver.cpp
  src/ws_server.cpp
)
target_include_directories(fastlio2 PUBLIC include)
target_link_libraries(fastlio2 PUBLIC Threads::Threads Boost::system yaml-cpp Eigen3::Eigen)

if(FASTLIO2_USE_PCL)
  target_link_libraries(fastlio2 PUBLIC ${PCL_LIBRARIES})
  target_compile_definitions(fastlio2 PUBLIC FASTLIO2_USE_PCL)
endif()

if(USE_UNITREE_SDK)
  target_link_libraries(fastlio2 PUBLIC unilidar_sdk2)
endif()

# ---- App: object_detection_pcl_live ----
add_executable(object_detection_pcl_live
  apps/object_detection_pcl_live.cpp
)
target_link_libraries(object_detection_pcl_live PRIVATE fastlio2)
target_compile_definitions(object_detection_pcl_live PRIVATE FASTLIO2_USE_PCL)

# ---- App: object_detection_windowed_live ----
add_executable(object_detection_windowed_live
  apps/object_detection_windowed_live.cpp
)
target_link_libraries(object_detection_windowed_live PRIVATE fastlio2)
target_compile_definitions(object_detection_windowed_live PRIVATE FASTLIO2_USE_PCL)


