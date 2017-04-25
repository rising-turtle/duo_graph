# Install script for directory: /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/apps/g2o_viewer

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
  IF(EXISTS "$ENV{DESTDIR}/usr/local/bin/g2o_viewer" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/bin/g2o_viewer")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}/usr/local/bin/g2o_viewer"
         RPATH "")
  ENDIF()
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/bin/g2o_viewer")
FILE(INSTALL DESTINATION "/usr/local/bin" TYPE EXECUTABLE FILES "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/bin/g2o_viewer")
  IF(EXISTS "$ENV{DESTDIR}/usr/local/bin/g2o_viewer" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/bin/g2o_viewer")
    FILE(RPATH_REMOVE
         FILE "$ENV{DESTDIR}/usr/local/bin/g2o_viewer")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/bin/g2o_viewer")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/g2o/apps/g2o_viewer/g2o_qglviewer.h;/usr/local/include/g2o/apps/g2o_viewer/stream_redirect.h;/usr/local/include/g2o/apps/g2o_viewer/main_window.h;/usr/local/include/g2o/apps/g2o_viewer/primitives.h;/usr/local/include/g2o/apps/g2o_viewer/gui_hyper_graph_action.h")
FILE(INSTALL DESTINATION "/usr/local/include/g2o/apps/g2o_viewer" TYPE FILE FILES
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/apps/g2o_viewer/g2o_qglviewer.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/apps/g2o_viewer/stream_redirect.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/apps/g2o_viewer/main_window.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/apps/g2o_viewer/primitives.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/apps/g2o_viewer/gui_hyper_graph_action.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

