set(Groovy_WS ${CMAKE_CURRENT_SOURCE_DIR}/../../../../)
set(RGBD_INCLUDE_DIRS ${Groovy_WS}/rgbdslam/src)
set(RGBD_LIBRARY_DIRS ${Groovy_WS}/rgbdslam/lib)
set(RGBD_LIBRARIES rgbdslam2)

set(MINET_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../minet)
set(MINET_LIBRARIES minet)

set(MISC_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../misc)
set(MISC_LIBRARIES misc)

set(OCTOWRAP_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../octowrapper)
set(OCTOWRAP_LIBRARIES octowrapper)


add_definitions(-DDO_FEATURE_OPTIMIZATION)
add_definitions(-DUSE_ICP_CODE)
include_directories(${RGBD_INCLUDE_DIRS} ${MINET_INCLUDE_DIRS} ${MISC_INCLUDE_DIRS} ${OCTOWRAP_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR} ${Groovy_WS}/rgbdslam/srv_gen/cpp/include)
link_directories(${RGBD_LIBRARY_DIRS} ${LIBRARY_OUTPUT_PATH})

#find_package(libg2o REQUIRED)
set(${libg2o_INCLUDE_DIRS} /opt/ros/groovy/include)
set(${libg2o_LIBRARY_DIRS} /opt/ros/groovy/lib)
set(${libg2o_LIBRARIES} g2o_core g2o_types_slam3d g2o_solver_cholmod g2o_solver_pcg g2o_solver_csparse g2o_stuff g2o_csparse_extension)
 
include_directories(${libg2o_INCLUDE_DIRS})
link_directories(${libg2o_LIBRARY_DIRS})

set(LINKS ${RGBD_LIBRARIES} ${OCTOWRAP_LIBRARIES} ${MINET_LIBRARIES} ${MISC_LIBRARIES}  ${libg2o_LIBRARIES} cholmod cxsparse ${QT_LIBRARIES} ${QT_QTOPENGL_LIBRARY} ${GLUT_LIBRARY} ${OPENGL_LIBRARY} ${OpenCV_LIBS} gicp)


set(OURQT_MOC OpenniWrapper.h NodeWrapper.h GraphWrapper.h)
QT4_WRAP_CPP(OURQT_MOC_HPP ${OURQT_MOC})
set(submap_SRCS NodeWrapper.cpp GraphWrapper.cpp OpenniWrapper.cpp  SubmapStore.cpp DuoGraph.cpp SubmapNode.cpp SubmapGraph.cpp Submap.cpp paramSrvMi.cpp)

rosbuild_add_executable(mainDuoSlam mainDuoSlam.cpp ${submap_SRCS} ${OURQT_MOC_HPP})
target_link_libraries(mainDuoSlam pcl_ros_tf g2o_core g2o_types_slam3d ${LINKS})

rosbuild_add_library(submap ${submap_SRCS} ${OURQT_MOC_HPP})
target_link_libraries(submap pcl_ros_tf ${LINKS})

rosbuild_add_executable(mainSubmapReader mainSubmapReader.cpp)
target_link_libraries(mainSubmapReader submap rt)

rosbuild_add_executable(mainMapBuilderOffline mainMapBuilderOffline.cpp)
target_link_libraries(mainMapBuilderOffline submap rt)

rosbuild_add_executable(mainMapBuilderROS mainMapBuilderROS.cpp)
target_link_libraries(mainMapBuilderROS submap rt)

rosbuild_add_executable(mainMapBuilderSocket mainMapBuilderSocket.cpp)
target_link_libraries(mainMapBuilderSocket submap rt)

rosbuild_add_executable(mainOriRgbdslam mainOriRgbdslam.cpp)
target_link_libraries(mainOriRgbdslam ${RGBD_LIBRARIES})



