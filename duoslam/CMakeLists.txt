set(ROS_COMPILE_FLAGS ${ROS_COMPILE_FLAGS} -fopenmp)

cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)
# set(CMAKE_CXX_FLAGS "-O3")
# set(ROS_BUILD_TYPE Debug)
rosbuild_init()

set(CMAKE_CXX_FLAGS "-O3 -w")
IF ("${ROS_PARALLEL_JOBS}" EQUAL "")
  set(ROS_PARALLEL_JOBS 1)
endif ("${ROS_PARALLEL_JOBS}" EQUAL "")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()


##############################################################################
# Required 3rd party
##############################################################################

#############################
# Qt ########################
#############################
# http://qtnode.net/wiki/Qt4_with_cmake
find_package(Qt4 REQUIRED)
SET(QT_USE_QTXML TRUE)
SET(QT_USE_QTOPENGL TRUE)
SET(QT_USE_QT3SUPPORT TRUE)
# ADD_DEFINITIONS(-DQT_NO_KEYWORDS)
include(${QT_USE_FILE})
include_directories(${Qt4_INCLUDE_DIRS})

find_package(VTK)
if(${VTK_FOUND})
    message("VTK FOUND!")
    include_directories(${VTK_INCLUDE_DIRS})
    link_directories(${VTK_LIBRARY_DIRS})
else(${VTK_FOUND})
    message("VTK NOT FOUND!")
    include_directories(${VTK_INC_DIR})
    link_directories(/usr/lib)
endif(${VTK_FOUND})

############################
#pcl
############################

set(Eigen_INCLUDE_DIRS /usr/include/eigen3-old)
include_directories(${Eigen_INCLUDE_DIRS})

# set(PCL_DIR /home/davidz/work/3rdproject/pcl-1.7.0/build)
# find_package(PCL 1.6 REQUIRED)
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})

#  set(EXTERN_DIR ${CMAKE_SOURCE_DIR}/external)
set(EXTERN_DIR /home/davidz/work/ros/groovy/catkin_ws/src/RT_SLAM/external)

###########################
# Octomap
###########################
set(OCTOMAP_INCLUDE ${EXTERN_DIR}/octomap/octovis/include 
    ${EXTERN_DIR}/octomap/octomap/include
    ${EXTERN_DIR}/octomap/dynamicEDT3D/include
    )
set(OCTOMAP_LIB_DIR ${EXTERN_DIR}/octomap/lib )
set(OCTOMAP_LIBS octomath octomap octovis dynamicedt3d )
include_directories(${OCTOMAP_INCLUDE})
include_directories(${PROJECT_SOURCE_DIR}/src)
link_directories(${OCTOMAP_LIB_DIR})
message("OCTOMAP INCLUDE ${OCTOMAP_INCLUDE}")

############################
# qglviewer.h
############################
# include_directories(/usr/include/QGLViewer)

# /usr/include/QGLViewer
# This is necessary as all ui files etc will get dumped in the bottom of then binary directory. 
include_directories(${CMAKE_CURRENT_BINARY_DIR} ${QT_QTOPENGL_INCLUDE_DIR} 
    ${EXTERN_DIR}/octomap/octovis/src/extern/QGLViewer
    external/gicp/ann_1.1.1/include/ANN external/gicp/ann_1.1.1/include external)
 	
LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})


set(USE_GICP 1)
#############################
# GICP ######################
#############################
IF (${USE_GICP})
	SET(gicp_build_path ${CMAKE_SOURCE_DIR}/external/gicp/ann_1.1.1)
  FILE(MAKE_DIRECTORY ${gicp_build_path}/lib/)
  MESSAGE("\n------------------------------------------------------------------\n")
	MESSAGE("Compiling GICP...")
	EXECUTE_PROCESS(
	    COMMAND make ${ROS_PARALLEL_JOBS} linux-g++
	    WORKING_DIRECTORY ${gicp_build_path}
	    RESULT_VARIABLE gicp_return
    )
    MESSAGE("\n------------------------------------------------------------------\n")
	
	#on error
	IF (NOT ${gicp_return} EQUAL 0)
		MESSAGE(FATAL_ERROR "GICP cannot be compiled. Returned: ${gicp_return}")
	ENDIF (NOT ${gicp_return} EQUAL 0)
	
	FILE(COPY ${gicp_build_path}/lib/libANN.a DESTINATION ${CMAKE_SOURCE_DIR}/lib/)
ENDIF (${USE_GICP})

#############################
# OpenCV ####################
#############################
find_package(OpenCV REQUIRED)
message("OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
include_directories(${OpenCV_INCLUDE_DIRS} /usr/include/suitesparse/)
link_directories(${OpenCV_LIBRARY_DIRS})

#############################
# Eigen ####################
#############################
find_package(Eigen)
IF(Eigen_FOUND)
  include_directories(${EIGEN_INCLUDE_DIRS})
  add_definitions(${EIGEN_DEFINITIONS})
ELSE(Eigen_FOUND)
  MESSAGE("Eigen package was not found. This is OK FOR ROS ELECTRIC, bad for fuerte\n")
ENDIF(Eigen_FOUND)

##############################################################################
# Some QT Magic (MOC Creation)
##############################################################################

#QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
#QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
#QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

if(${USE_GICP})
    set(ROS_COMPILE_FLAGS ${ROS_COMPILE_FLAGS} -fpermissive)
    rosbuild_add_library(gicp external/gicp/bfgs_funcs.cpp external/gicp/gicp.cpp external/gicp/optimize.cpp external/gicp/scan.cpp external/gicp/transform.cpp)
endif(${USE_GICP})

add_subdirectory(src)
