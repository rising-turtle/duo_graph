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
include g2o/apps/g2o_interactive/CMakeFiles/g2o_online_application.dir/depend.make

# Include the progress variables for this target.
include g2o/apps/g2o_interactive/CMakeFiles/g2o_online_application.dir/progress.make

# Include the compile flags for this target's objects.
include g2o/apps/g2o_interactive/CMakeFiles/g2o_online_application.dir/flags.make

g2o/apps/g2o_interactive/CMakeFiles/g2o_online_application.dir/g2o_online.cpp.o: g2o/apps/g2o_interactive/CMakeFiles/g2o_online_application.dir/flags.make
g2o/apps/g2o_interactive/CMakeFiles/g2o_online_application.dir/g2o_online.cpp.o: ../g2o/apps/g2o_interactive/g2o_online.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object g2o/apps/g2o_interactive/CMakeFiles/g2o_online_application.dir/g2o_online.cpp.o"
	cd /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/build/g2o/apps/g2o_interactive && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/g2o_online_application.dir/g2o_online.cpp.o -c /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/apps/g2o_interactive/g2o_online.cpp

g2o/apps/g2o_interactive/CMakeFiles/g2o_online_application.dir/g2o_online.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/g2o_online_application.dir/g2o_online.cpp.i"
	cd /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/build/g2o/apps/g2o_interactive && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/apps/g2o_interactive/g2o_online.cpp > CMakeFiles/g2o_online_application.dir/g2o_online.cpp.i

g2o/apps/g2o_interactive/CMakeFiles/g2o_online_application.dir/g2o_online.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/g2o_online_application.dir/g2o_online.cpp.s"
	cd /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/build/g2o/apps/g2o_interactive && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/apps/g2o_interactive/g2o_online.cpp -o CMakeFiles/g2o_online_application.dir/g2o_online.cpp.s

g2o/apps/g2o_interactive/CMakeFiles/g2o_online_application.dir/g2o_online.cpp.o.requires:
.PHONY : g2o/apps/g2o_interactive/CMakeFiles/g2o_online_application.dir/g2o_online.cpp.o.requires

g2o/apps/g2o_interactive/CMakeFiles/g2o_online_application.dir/g2o_online.cpp.o.provides: g2o/apps/g2o_interactive/CMakeFiles/g2o_online_application.dir/g2o_online.cpp.o.requires
	$(MAKE) -f g2o/apps/g2o_interactive/CMakeFiles/g2o_online_application.dir/build.make g2o/apps/g2o_interactive/CMakeFiles/g2o_online_application.dir/g2o_online.cpp.o.provides.build
.PHONY : g2o/apps/g2o_interactive/CMakeFiles/g2o_online_application.dir/g2o_online.cpp.o.provides

g2o/apps/g2o_interactive/CMakeFiles/g2o_online_application.dir/g2o_online.cpp.o.provides.build: g2o/apps/g2o_interactive/CMakeFiles/g2o_online_application.dir/g2o_online.cpp.o

# Object files for target g2o_online_application
g2o_online_application_OBJECTS = \
"CMakeFiles/g2o_online_application.dir/g2o_online.cpp.o"

# External object files for target g2o_online_application
g2o_online_application_EXTERNAL_OBJECTS =

../bin/g2o_online: g2o/apps/g2o_interactive/CMakeFiles/g2o_online_application.dir/g2o_online.cpp.o
../bin/g2o_online: ../lib/libg2o_interactive.so
../bin/g2o_online: ../lib/libg2o_types_slam2d.so
../bin/g2o_online: ../lib/libg2o_types_slam3d.so
../bin/g2o_online: /usr/lib/i386-linux-gnu/libGL.so
../bin/g2o_online: ../lib/libg2o_math_groups.so
../bin/g2o_online: ../lib/libg2o_solver_cholmod.so
../bin/g2o_online: ../lib/libg2o_core.so
../bin/g2o_online: ../lib/libg2o_stuff.so
../bin/g2o_online: /usr/lib/libcholmod.so
../bin/g2o_online: /usr/lib/libamd.so
../bin/g2o_online: ../lib/libg2o_interface.so
../bin/g2o_online: ../lib/libg2o_parser.so
../bin/g2o_online: g2o/apps/g2o_interactive/CMakeFiles/g2o_online_application.dir/build.make
../bin/g2o_online: g2o/apps/g2o_interactive/CMakeFiles/g2o_online_application.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../../../../bin/g2o_online"
	cd /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/build/g2o/apps/g2o_interactive && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/g2o_online_application.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
g2o/apps/g2o_interactive/CMakeFiles/g2o_online_application.dir/build: ../bin/g2o_online
.PHONY : g2o/apps/g2o_interactive/CMakeFiles/g2o_online_application.dir/build

g2o/apps/g2o_interactive/CMakeFiles/g2o_online_application.dir/requires: g2o/apps/g2o_interactive/CMakeFiles/g2o_online_application.dir/g2o_online.cpp.o.requires
.PHONY : g2o/apps/g2o_interactive/CMakeFiles/g2o_online_application.dir/requires

g2o/apps/g2o_interactive/CMakeFiles/g2o_online_application.dir/clean:
	cd /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/build/g2o/apps/g2o_interactive && $(CMAKE_COMMAND) -P CMakeFiles/g2o_online_application.dir/cmake_clean.cmake
.PHONY : g2o/apps/g2o_interactive/CMakeFiles/g2o_online_application.dir/clean

g2o/apps/g2o_interactive/CMakeFiles/g2o_online_application.dir/depend:
	cd /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/apps/g2o_interactive /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/build /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/build/g2o/apps/g2o_interactive /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/build/g2o/apps/g2o_interactive/CMakeFiles/g2o_online_application.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : g2o/apps/g2o_interactive/CMakeFiles/g2o_online_application.dir/depend

