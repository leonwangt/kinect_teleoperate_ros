# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.
cmake_minimum_required(VERSION 3.14.0)

set(CMAKE_POLICY_DEFAULT_CMP0063 NEW)

project(kinect_teleoperate LANGUAGES C CXX  VERSION 1.4)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 99)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE Debug)

# Eigen
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

message(STATUS "Current system architecture: ${CMAKE_SYSTEM_PROCESSOR}")

set(MuJoCo_Include_Dir ${CMAKE_CURRENT_SOURCE_DIR}/../mujoco-3.1.5/include)
set(MuJoCo_Library_Dir ${CMAKE_CURRENT_SOURCE_DIR}/../mujoco-3.1.5/lib)
set(MuJoCo_Libraries mujoco)

link_directories(
    ${MuJoCo_Library_Dir}
    ${hx_idl_cpp_Include_Dir}
)

list(APPEND CMAKE_PREFIX_PATH "/usr/local/lib/cmake/lcm")
# list(APPEND CMAKE_PREFIX_PATH "/usr/local/lib")
# list(APPEND CMAKE_PREFIX_PATH "/usr/local/include")
find_path(LCM_INCLUDE_DIR lcm/lcm.h)
find_library(LCM_LIBRARY lcm)

set(ROS_INCLUDE_DIR "/opt/ros/noetic/include/")
set(UNITREE_SDK_INCLUDE_DIR "/usr/local/include")

## Import Unitree SDK2 library
message(STATUS "Importing: ${UNITREE_SDK_LIB}")

set(UNITREE_SDK_DDS_PATH "/usr/local/lib")
find_library(UNITREE_SDK_LIB unitree_sdk2  PATHS ${UNITREE_SDK_DDS_PATH} NO_DEFAULT_PATH)

if (NOT UNITREE_SDK_LIB)
    message(FATAL_ERROR "Unitree SDK library for the architecture is not found")
else ()
    message(STATUS "Unitree SDK library found at: ${UNITREE_SDK_LIB}")
endif ()

## Thread
message(STATUS "Importing: Threads")

find_package(Threads REQUIRED)
include_directories(/usr/local/include/ddscxx)

message(STATUS "Found: Threads")


# #msg
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  message_generation
)

# add_message_files(
#   FILES
#   Rotation.msg
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )
# catkin_package(
#   CATKIN_DEPENDS roscpp geometry_msgs std_msgs message_runtime
# )

## dds
message(STATUS "Importing: ${DDSC_SDK_LIB}")

find_library(DDSC_SDK_LIB ddsc  PATHS ${UNITREE_SDK_DDS_PATH} NO_DEFAULT_PATH)

if (NOT DDSC_SDK_LIB)
    message(FATAL_ERROR "ddsc library for the architecture is not found")
else ()
    message(STATUS "ddsc library found at: ${DDSC_SDK_LIB}")
endif ()

add_library(ddsc SHARED IMPORTED GLOBAL)
set_target_properties(ddsc PROPERTIES
        IMPORTED_LOCATION ${DDSC_SDK_LIB})
set_target_properties(ddsc PROPERTIES IMPORTED_NO_SONAME TRUE)
target_link_libraries(ddsc INTERFACE Threads::Threads)
target_link_directories(ddsc INTERFACE ${UNITREE_SDK_DDS_PATH})
target_include_directories(ddsc INTERFACE
        ${UNITREE_SDK_INCLUDE_DIR})
        
#--------------------------------------------------------#
message(STATUS "Importing: ${DDSCXX_SDK_LIB}")

find_library(DDSCXX_SDK_LIB ddscxx  PATHS ${UNITREE_SDK_DDS_PATH} NO_DEFAULT_PATH)

if (NOT DDSCXX_SDK_LIB)
    message(FATAL_ERROR "ddscxx library for the architecture is not found")
else ()
    message(STATUS "ddslibrary found at: ${DDSCXX_SDK_LIB}")
endif ()

add_library(ddscxx SHARED IMPORTED GLOBAL)
set_target_properties(ddscxx PROPERTIES
        IMPORTED_LOCATION ${DDSCXX_SDK_LIB})
set_target_properties(ddscxx PROPERTIES IMPORTED_NO_SONAME TRUE)
target_link_libraries(ddscxx INTERFACE Threads::Threads)
target_link_directories(ddscxx INTERFACE ${UNITREE_SDK_DDS_PATH})
target_include_directories(ddscxx INTERFACE
        ${UNITREE_SDK_INCLUDE_DIR}
        ${UNITREE_SDK_INCLUDE_DIR}/ddscxx)

## Add unitree sdk
add_library(unitree_sdk2 STATIC IMPORTED GLOBAL)
set_target_properties(unitree_sdk2 PROPERTIES
        IMPORTED_LOCATION ${UNITREE_SDK_LIB})
target_link_libraries(unitree_sdk2 INTERFACE ddsc ddscxx Threads::Threads)
target_include_directories(unitree_sdk2 INTERFACE
        ${UNITREE_SDK_INCLUDE_DIR})


add_executable(kinect_teleoperate  main.cpp)

set_target_properties(kinect_teleoperate PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

target_include_directories(kinect_teleoperate 
    PRIVATE 
    ../sample_helper_includes 
    ${MuJoCo_Include_Dir}
    include/
    ${UNITREE_SDK_INCLUDE_DIR} 
    ${ROS_INCLUDE_DIR}  
    ${catkin_INCLUDE_DIRS}
)

# Dependencies of this library
target_link_libraries(kinect_teleoperate PRIVATE 
    k4a
    k4abt
    k4arecord
    window_controller_3d::window_controller_3d
    glfw::glfw
    ${MuJoCo_Libraries}
    Eigen3::Eigen
    lcm
    unitree_sdk2
    ${catkin_LIBRARIES}
)

