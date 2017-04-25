set(Groovy_WS /home/davidz/work/ros/groovy/catkin_ws/src)
set(RGBD_INCLUDE_DIRS ${Groovy_WS}/rgbdslam/src)
set(RGBD_LIBRARY_DIRS ${Groovy_WS}/rgbdslam/lib)
set(RGBD_LIBRARIES rgbdslam2)

add_definitions(-DDO_FEATURE_OPTIMIZATION)
include_directories(${RGBD_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} ${Groovy_WS}/rgbdslam/srv_gen/cpp/include)
link_directories(${RGBD_LIBRARY_DIRS})
# link_directories(${CMAKE_LIBRARY_OUTPUT_DIR})

find_package(libg2o REQUIRED)
include_directories(${libg2o_INCLUDE_DIRS})
link_directories(${libg2o_LIBRARY_DIRS})

set(LINKS ${libg2o_LIBRARIES} cholmod cxsparse ${QT_LIBRARIES} ${QT_QTOPENGL_LIBRARY} ${GLUT_LIBRARY} ${OPENGL_LIBRARY} ${OpenCV_LIBS})

set(GLOBAL_CPP ../globaldef.cpp ../colortable.cpp ../timestamp.cpp)
set(OURQT_MOC OpenniWrapper.h NodeWrapper.h GraphWrapper.h)
QT4_WRAP_CPP(OURQT_MOC_HPP ${OURQT_MOC})
set(submap_SRCS NodeWrapper.cpp GraphWrapper.cpp OpenniWrapper.cpp slam_globaldef.cpp SubmapStore.cpp DuoGraph.cpp SubmapNode.cpp SubmapGraph.cpp Submap.cpp)
rosbuild_add_executable(submap1 test_submap.cpp ${submap_SRCS} ${OURQT_MOC_HPP} ${GLOBAL_CPP})
target_link_libraries(submap1 ${RGBD_LIBRARIES} octowrapper pcl_ros_tf ${LINKS})

rosbuild_add_library(submap ${submap_SRCS} ${OURQT_MOC_HPP})
target_link_libraries(submap ${RGBD_LIBRARIES} octowrapper pcl_ros_tf ${LINKS})
rosbuild_add_executable(submap2 test_submap2.cpp ${GLOBAL_CPP})
target_link_libraries(submap2 submap rt)

rosbuild_add_executable(submap3 test_submap3.cpp ${GLOBAL_CPP})
target_link_libraries(submap3 submap rt)

rosbuild_add_executable(rgbdslam ori_rgbd.cpp)
target_link_libraries(rgbdslam ${RGBD_LIBRARIES})



