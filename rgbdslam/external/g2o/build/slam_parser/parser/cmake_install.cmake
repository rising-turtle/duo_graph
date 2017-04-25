# Install script for directory: /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/slam_parser/parser

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Release")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}/usr/local/lib/libg2o_parser.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/lib/libg2o_parser.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}/usr/local/lib/libg2o_parser.so"
         RPATH "")
  ENDIF()
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/libg2o_parser.so")
FILE(INSTALL DESTINATION "/usr/local/lib" TYPE SHARED_LIBRARY FILES "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/lib/libg2o_parser.so")
  IF(EXISTS "$ENV{DESTDIR}/usr/local/lib/libg2o_parser.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/lib/libg2o_parser.so")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/lib/libg2o_parser.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/slam_parser/parser/commands.h;/usr/local/include/slam_parser/parser/bison_parser.h;/usr/local/include/slam_parser/parser/driver.h;/usr/local/include/slam_parser/parser/FlexLexer.h;/usr/local/include/slam_parser/parser/scanner.h;/usr/local/include/slam_parser/parser/slam_context.h;/usr/local/include/slam_parser/parser/location.hh;/usr/local/include/slam_parser/parser/stack.hh;/usr/local/include/slam_parser/parser/position.hh")
FILE(INSTALL DESTINATION "/usr/local/include/slam_parser/parser" TYPE FILE FILES
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/slam_parser/parser/commands.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/slam_parser/parser/bison_parser.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/slam_parser/parser/driver.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/slam_parser/parser/FlexLexer.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/slam_parser/parser/scanner.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/slam_parser/parser/slam_context.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/slam_parser/parser/location.hh"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/slam_parser/parser/stack.hh"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/slam_parser/parser/position.hh"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

