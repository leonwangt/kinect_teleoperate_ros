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

# Include any dependencies generated for this target.
include _deps/nlohmann_json-build/test/CMakeFiles/test-algorithms.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include _deps/nlohmann_json-build/test/CMakeFiles/test-algorithms.dir/compiler_depend.make

# Include the progress variables for this target.
include _deps/nlohmann_json-build/test/CMakeFiles/test-algorithms.dir/progress.make

# Include the compile flags for this target's objects.
include _deps/nlohmann_json-build/test/CMakeFiles/test-algorithms.dir/flags.make

_deps/nlohmann_json-build/test/CMakeFiles/test-algorithms.dir/codegen:
.PHONY : _deps/nlohmann_json-build/test/CMakeFiles/test-algorithms.dir/codegen

_deps/nlohmann_json-build/test/CMakeFiles/test-algorithms.dir/src/unit-algorithms.cpp.o: _deps/nlohmann_json-build/test/CMakeFiles/test-algorithms.dir/flags.make
_deps/nlohmann_json-build/test/CMakeFiles/test-algorithms.dir/src/unit-algorithms.cpp.o: /home/wang/Desktop/Unitree_H1/kinect_teleoperate_ros/build/_deps/nlohmann_json-src/test/src/unit-algorithms.cpp
_deps/nlohmann_json-build/test/CMakeFiles/test-algorithms.dir/src/unit-algorithms.cpp.o: _deps/nlohmann_json-build/test/CMakeFiles/test-algorithms.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/wang/Desktop/Unitree_H1/kinect_teleoperate_ros/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object _deps/nlohmann_json-build/test/CMakeFiles/test-algorithms.dir/src/unit-algorithms.cpp.o"
	cd /home/wang/Desktop/Unitree_H1/kinect_teleoperate_ros/build/_deps/nlohmann_json-build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT _deps/nlohmann_json-build/test/CMakeFiles/test-algorithms.dir/src/unit-algorithms.cpp.o -MF CMakeFiles/test-algorithms.dir/src/unit-algorithms.cpp.o.d -o CMakeFiles/test-algorithms.dir/src/unit-algorithms.cpp.o -c /home/wang/Desktop/Unitree_H1/kinect_teleoperate_ros/build/_deps/nlohmann_json-src/test/src/unit-algorithms.cpp

_deps/nlohmann_json-build/test/CMakeFiles/test-algorithms.dir/src/unit-algorithms.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/test-algorithms.dir/src/unit-algorithms.cpp.i"
	cd /home/wang/Desktop/Unitree_H1/kinect_teleoperate_ros/build/_deps/nlohmann_json-build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wang/Desktop/Unitree_H1/kinect_teleoperate_ros/build/_deps/nlohmann_json-src/test/src/unit-algorithms.cpp > CMakeFiles/test-algorithms.dir/src/unit-algorithms.cpp.i

_deps/nlohmann_json-build/test/CMakeFiles/test-algorithms.dir/src/unit-algorithms.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/test-algorithms.dir/src/unit-algorithms.cpp.s"
	cd /home/wang/Desktop/Unitree_H1/kinect_teleoperate_ros/build/_deps/nlohmann_json-build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wang/Desktop/Unitree_H1/kinect_teleoperate_ros/build/_deps/nlohmann_json-src/test/src/unit-algorithms.cpp -o CMakeFiles/test-algorithms.dir/src/unit-algorithms.cpp.s

# Object files for target test-algorithms
test__algorithms_OBJECTS = \
"CMakeFiles/test-algorithms.dir/src/unit-algorithms.cpp.o"

# External object files for target test-algorithms
test__algorithms_EXTERNAL_OBJECTS = \
"/home/wang/Desktop/Unitree_H1/kinect_teleoperate_ros/build/_deps/nlohmann_json-build/test/CMakeFiles/doctest_main.dir/src/unit.cpp.o"

teleoperate_ros/test-algorithms: _deps/nlohmann_json-build/test/CMakeFiles/test-algorithms.dir/src/unit-algorithms.cpp.o
teleoperate_ros/test-algorithms: _deps/nlohmann_json-build/test/CMakeFiles/doctest_main.dir/src/unit.cpp.o
teleoperate_ros/test-algorithms: _deps/nlohmann_json-build/test/CMakeFiles/test-algorithms.dir/build.make
teleoperate_ros/test-algorithms: _deps/nlohmann_json-build/test/CMakeFiles/test-algorithms.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/wang/Desktop/Unitree_H1/kinect_teleoperate_ros/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../../teleoperate_ros/test-algorithms"
	cd /home/wang/Desktop/Unitree_H1/kinect_teleoperate_ros/build/_deps/nlohmann_json-build/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test-algorithms.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
_deps/nlohmann_json-build/test/CMakeFiles/test-algorithms.dir/build: teleoperate_ros/test-algorithms
.PHONY : _deps/nlohmann_json-build/test/CMakeFiles/test-algorithms.dir/build

_deps/nlohmann_json-build/test/CMakeFiles/test-algorithms.dir/clean:
	cd /home/wang/Desktop/Unitree_H1/kinect_teleoperate_ros/build/_deps/nlohmann_json-build/test && $(CMAKE_COMMAND) -P CMakeFiles/test-algorithms.dir/cmake_clean.cmake
.PHONY : _deps/nlohmann_json-build/test/CMakeFiles/test-algorithms.dir/clean

_deps/nlohmann_json-build/test/CMakeFiles/test-algorithms.dir/depend:
	cd /home/wang/Desktop/Unitree_H1/kinect_teleoperate_ros/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wang/Desktop/Unitree_H1/kinect_teleoperate_ros/src /home/wang/Desktop/Unitree_H1/kinect_teleoperate_ros/build/_deps/nlohmann_json-src/test /home/wang/Desktop/Unitree_H1/kinect_teleoperate_ros/build /home/wang/Desktop/Unitree_H1/kinect_teleoperate_ros/build/_deps/nlohmann_json-build/test /home/wang/Desktop/Unitree_H1/kinect_teleoperate_ros/build/_deps/nlohmann_json-build/test/CMakeFiles/test-algorithms.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : _deps/nlohmann_json-build/test/CMakeFiles/test-algorithms.dir/depend
