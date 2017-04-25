# Install script for directory: /home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/core

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
  IF(EXISTS "$ENV{DESTDIR}/usr/local/lib/libg2o_core.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/lib/libg2o_core.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}/usr/local/lib/libg2o_core.so"
         RPATH "")
  ENDIF()
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/libg2o_core.so")
FILE(INSTALL DESTINATION "/usr/local/lib" TYPE SHARED_LIBRARY FILES "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/lib/libg2o_core.so")
  IF(EXISTS "$ENV{DESTDIR}/usr/local/lib/libg2o_core.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/lib/libg2o_core.so")
    FILE(RPATH_REMOVE
         FILE "$ENV{DESTDIR}/usr/local/lib/libg2o_core.so")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/lib/libg2o_core.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/g2o/core/base_vertex.h;/usr/local/include/g2o/core/creators.h;/usr/local/include/g2o/core/hyper_graph.h;/usr/local/include/g2o/core/block_solver.h;/usr/local/include/g2o/core/optimizable_graph.h;/usr/local/include/g2o/core/factory.h;/usr/local/include/g2o/core/solver.h;/usr/local/include/g2o/core/matrix_structure.h;/usr/local/include/g2o/core/estimate_propagator.h;/usr/local/include/g2o/core/linear_solver.h;/usr/local/include/g2o/core/hyper_graph_action.h;/usr/local/include/g2o/core/base_unary_edge.h;/usr/local/include/g2o/core/openmp_mutex.h;/usr/local/include/g2o/core/batch_stats.h;/usr/local/include/g2o/core/marginal_covariance_cholesky.h;/usr/local/include/g2o/core/solver_factory.h;/usr/local/include/g2o/core/base_edge.h;/usr/local/include/g2o/core/base_binary_edge.h;/usr/local/include/g2o/core/graph_optimizer_sparse.h;/usr/local/include/g2o/core/hyper_dijkstra.h;/usr/local/include/g2o/core/structure_only_solver.h;/usr/local/include/g2o/core/sparse_block_matrix.h;/usr/local/include/g2o/core/solver_property.h;/usr/local/include/g2o/core/base_multi_edge.h;/usr/local/include/g2o/core/block_solver.hpp;/usr/local/include/g2o/core/base_binary_edge.hpp;/usr/local/include/g2o/core/base_unary_edge.hpp;/usr/local/include/g2o/core/base_vertex.hpp;/usr/local/include/g2o/core/sparse_block_matrix.hpp;/usr/local/include/g2o/core/base_multi_edge.hpp")
FILE(INSTALL DESTINATION "/usr/local/include/g2o/core" TYPE FILE FILES
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/core/base_vertex.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/core/creators.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/core/hyper_graph.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/core/block_solver.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/core/optimizable_graph.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/core/factory.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/core/solver.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/core/matrix_structure.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/core/estimate_propagator.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/core/linear_solver.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/core/hyper_graph_action.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/core/base_unary_edge.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/core/openmp_mutex.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/core/batch_stats.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/core/marginal_covariance_cholesky.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/core/solver_factory.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/core/base_edge.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/core/base_binary_edge.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/core/graph_optimizer_sparse.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/core/hyper_dijkstra.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/core/structure_only_solver.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/core/sparse_block_matrix.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/core/solver_property.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/core/base_multi_edge.h"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/core/block_solver.hpp"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/core/base_binary_edge.hpp"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/core/base_unary_edge.hpp"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/core/base_vertex.hpp"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/core/sparse_block_matrix.hpp"
    "/home/davidz/work/ros/groovy/catkin_ws/src/rgbdslam/external/g2o/g2o/core/base_multi_edge.hpp"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

