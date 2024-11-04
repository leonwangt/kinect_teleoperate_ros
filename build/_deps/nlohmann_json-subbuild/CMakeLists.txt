# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.31.0-rc2)

# We name the project and the target for the ExternalProject_Add() call
# to something that will highlight to the user what we are working on if
# something goes wrong and an error message is produced.

project(nlohmann_json-populate NONE)

include(ExternalProject)
ExternalProject_Add(nlohmann_json-populate
                     "UPDATE_DISCONNECTED" "False" "GIT_REPOSITORY" "https://github.com/nlohmann/json.git" "GIT_TAG" "v3.7.0"
                    SOURCE_DIR          "/home/wang/Desktop/Unitree_H1/kinect_teleoperate_ros/build/_deps/nlohmann_json-src"
                    BINARY_DIR          "/home/wang/Desktop/Unitree_H1/kinect_teleoperate_ros/build/_deps/nlohmann_json-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
)
