###########################
# Octomap Viewer
###########################
# set(VIEWER_DIR ${EXTERN_DIR}/octomap/octovis/include/octovis)
# set(VIEWER_SRC_DIR ${EXTERN_DIR}/octomap/octovis/src)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../misc/)

set(VIEWER_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/viewer)
set(VIEWER_DIR ${VIEWER_SRC_DIR})
set(viewer_SRCS
    ${VIEWER_SRC_DIR}/ViewerGui.cpp
    ${VIEWER_SRC_DIR}/ViewerWidget.cpp
    ${VIEWER_SRC_DIR}/ViewerSettings.cpp
    ${VIEWER_SRC_DIR}/ViewerSettingsPanel.cpp
    ${VIEWER_SRC_DIR}/ViewerSettingsPanelCamera.cpp
    ${VIEWER_SRC_DIR}/CameraFollowMode.cpp
    )
set(viewer_RES ${EXTERN_DIR}/octomap/octovis/src/icons.qrc)
set(viewer_MOC_HDRS 
    ${VIEWER_DIR}/ViewerGui.h
    ${VIEWER_DIR}/ViewerWidget.h
    ${VIEWER_DIR}/ViewerSettings.h
    ${VIEWER_DIR}/ViewerSettingsPanel.h
    ${VIEWER_DIR}/ViewerSettingsPanelCamera.h
    ${VIEWER_DIR}/CameraFollowMode.h
    )
include_directories(${VIEWER_SRC_DIR})
QT4_WRAP_CPP(viewer_MOC_SRCS ${viewer_MOC_HDRS})
set(viewer_UIS
    ${VIEWER_DIR}/ViewerGui.ui
    ${VIEWER_DIR}/ViewerSettings.ui
    ${VIEWER_DIR}/ViewerSettingsPanel.ui
    ${VIEWER_DIR}/ViewerSettingsPanelCamera.ui
    )

SET(LIBS_LINK GL GLU gsl gslcblas  ${OCTOMAP_LIBS} ${QT_LIBRARIES} ${QT_QTOPENGL_LIBRARY} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
file(GLOB OURQT_MOC *.h)
QT4_WRAP_CPP(OURQT_MOC_HPP ${OURQT_MOC})
# file(GLOB OURSRCS src/*.cpp)
QT4_WRAP_UI(viewer_UIS_H ${viewer_UIS})
set(VIEWER_FILES ${viewer_SRCS} ${viewer_MOC_SRCS} ${viewer_UIS_H} ${viewer_RES})
set(OURSRCS bag_listener.cpp OctoVizWrapper.cpp ColorOctreeImpl.cpp FileReader.cpp octo_globaldef.cpp)

rosbuild_add_library(octowrapper ${OURSRCS} ${OURQT_MOC_HPP} ${VIEWER_FILES})
target_link_libraries(octowrapper ${LIBS_LINK})

