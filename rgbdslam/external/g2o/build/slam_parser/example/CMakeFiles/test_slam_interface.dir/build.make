# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/build

# Include any dependencies generated for this target.
include slam_parser/example/CMakeFiles/test_slam_interface.dir/depend.make

# Include the progress variables for this target.
include slam_parser/example/CMakeFiles/test_slam_interface.dir/progress.make

# Include the compile flags for this target's objects.
include slam_parser/example/CMakeFiles/test_slam_interface.dir/flags.make

slam_parser/example/CMakeFiles/test_slam_interface.dir/test_slam_interface.cpp.o: slam_parser/example/CMakeFiles/test_slam_interface.dir/flags.make
slam_parser/example/CMakeFiles/test_slam_interface.dir/test_slam_interface.cpp.o: ../slam_parser/example/test_slam_interface.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object slam_parser/example/CMakeFiles/test_slam_interface.dir/test_slam_interface.cpp.o"
	cd /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/build/slam_parser/example && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test_slam_interface.dir/test_slam_interface.cpp.o -c /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/slam_parser/example/test_slam_interface.cpp

slam_parser/example/CMakeFiles/test_slam_interface.dir/test_slam_interface.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_slam_interface.dir/test_slam_interface.cpp.i"
	cd /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/build/slam_parser/example && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/slam_parser/example/test_slam_interface.cpp > CMakeFiles/test_slam_interface.dir/test_slam_interface.cpp.i

slam_parser/example/CMakeFiles/test_slam_interface.dir/test_slam_interface.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_slam_interface.dir/test_slam_interface.cpp.s"
	cd /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/build/slam_parser/example && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/slam_parser/example/test_slam_interface.cpp -o CMakeFiles/test_slam_interface.dir/test_slam_interface.cpp.s

slam_parser/example/CMakeFiles/test_slam_interface.dir/test_slam_interface.cpp.o.requires:
.PHONY : slam_parser/example/CMakeFiles/test_slam_interface.dir/test_slam_interface.cpp.o.requires

slam_parser/example/CMakeFiles/test_slam_interface.dir/test_slam_interface.cpp.o.provides: slam_parser/example/CMakeFiles/test_slam_interface.dir/test_slam_interface.cpp.o.requires
	$(MAKE) -f slam_parser/example/CMakeFiles/test_slam_interface.dir/build.make slam_parser/example/CMakeFiles/test_slam_interface.dir/test_slam_interface.cpp.o.provides.build
.PHONY : slam_parser/example/CMakeFiles/test_slam_interface.dir/test_slam_interface.cpp.o.provides

slam_parser/example/CMakeFiles/test_slam_interface.dir/test_slam_interface.cpp.o.provides.build: slam_parser/example/CMakeFiles/test_slam_interface.dir/test_slam_interface.cpp.o

# Object files for target test_slam_interface
test_slam_interface_OBJECTS = \
"CMakeFiles/test_slam_interface.dir/test_slam_interface.cpp.o"

# External object files for target test_slam_interface
test_slam_interface_EXTERNAL_OBJECTS =

../bin/test_slam_interface: slam_parser/example/CMakeFiles/test_slam_interface.dir/test_slam_interface.cpp.o
../bin/test_slam_interface: ../lib/libg2o_example.so
../bin/test_slam_interface: ../lib/libg2o_interface.so
../bin/test_slam_interface: ../lib/libg2o_parser.so
../bin/test_slam_interface: slam_parser/example/CMakeFiles/test_slam_interface.dir/build.make
../bin/test_slam_interface: slam_parser/example/CMakeFiles/test_slam_interface.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../../../bin/test_slam_interface"
	cd /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/build/slam_parser/example && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_slam_interface.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
slam_parser/example/CMakeFiles/test_slam_interface.dir/build: ../bin/test_slam_interface
.PHONY : slam_parser/example/CMakeFiles/test_slam_interface.dir/build

slam_parser/example/CMakeFiles/test_slam_interface.dir/requires: slam_parser/example/CMakeFiles/test_slam_interface.dir/test_slam_interface.cpp.o.requires
.PHONY : slam_parser/example/CMakeFiles/test_slam_interface.dir/requires

slam_parser/example/CMakeFiles/test_slam_interface.dir/clean:
	cd /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/build/slam_parser/example && $(CMAKE_COMMAND) -P CMakeFiles/test_slam_interface.dir/cmake_clean.cmake
.PHONY : slam_parser/example/CMakeFiles/test_slam_interface.dir/clean

slam_parser/example/CMakeFiles/test_slam_interface.dir/depend:
	cd /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/slam_parser/example /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/build /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/build/slam_parser/example /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/build/slam_parser/example/CMakeFiles/test_slam_interface.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : slam_parser/example/CMakeFiles/test_slam_interface.dir/depend

