# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.31

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/wang/Desktop/Unitree_H1/kinect_teleoperate_ros/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wang/Desktop/Unitree_H1/kinect_teleoperate_ros/build

# Utility rule file for Nightly.

# Include any custom commands dependencies for this target.
include _deps/nlohmann_json-build/CMakeFiles/Nightly.dir/compiler_depend.make

# Include the progress variables for this target.
include _deps/nlohmann_json-build/CMakeFiles/Nightly.dir/progress.make

_deps/nlohmann_json-build/CMakeFiles/Nightly:
	cd /home/wang/Desktop/Unitree_H1/kinect_teleoperate_ros/build/_deps/nlohmann_json-build && /usr/local/bin/ctest -D Nightly

_deps/nlohmann_json-build/CMakeFiles/Nightly.dir/codegen:
.PHONY : _deps/nlohmann_json-build/CMakeFiles/Nightly.dir/codegen

Nightly: _deps/nlohmann_json-build/CMakeFiles/Nightly
Nightly: _deps/nlohmann_json-build/CMakeFiles/Nightly.dir/build.make
.PHONY : Nightly

# Rule to build all files generated by this target.
_deps/nlohmann_json-build/CMakeFiles/Nightly.dir/build: Nightly
.PHONY : _deps/nlohmann_json-build/CMakeFiles/Nightly.dir/build

_deps/nlohmann_json-build/CMakeFiles/Nightly.dir/clean:
	cd /home/wang/Desktop/Unitree_H1/kinect_teleoperate_ros/build/_deps/nlohmann_json-build && $(CMAKE_COMMAND) -P CMakeFiles/Nightly.dir/cmake_clean.cmake
.PHONY : _deps/nlohmann_json-build/CMakeFiles/Nightly.dir/clean

_deps/nlohmann_json-build/CMakeFiles/Nightly.dir/depend:
	cd /home/wang/Desktop/Unitree_H1/kinect_teleoperate_ros/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wang/Desktop/Unitree_H1/kinect_teleoperate_ros/src /home/wang/Desktop/Unitree_H1/kinect_teleoperate_ros/build/_deps/nlohmann_json-src /home/wang/Desktop/Unitree_H1/kinect_teleoperate_ros/build /home/wang/Desktop/Unitree_H1/kinect_teleoperate_ros/build/_deps/nlohmann_json-build /home/wang/Desktop/Unitree_H1/kinect_teleoperate_ros/build/_deps/nlohmann_json-build/CMakeFiles/Nightly.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : _deps/nlohmann_json-build/CMakeFiles/Nightly.dir/depend

