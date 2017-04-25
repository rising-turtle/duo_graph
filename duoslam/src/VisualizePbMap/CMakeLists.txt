# PROJECT( EDPbmap_proj )

# cmake_minimum_required(VERSION 2.8)

# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#OPENNI
set(OPENNI_INCLUDE_DIRS "/usr/include/ni")
set(OPENNI_LIB_DIR "/usr/lib")
set(OPENNI_LIBS OpenNI)

include_directories(${Eigen_INCLUDE_DIRS})

# find_package(Eigen)
# MESSAGE(STATUS "Eigen include directions: ${EIGEN_INCLUDE_DIRS}")
# MESSAGE(STATUS "Eigen library directions: ${EIGEN_LIBRARY_DIRS}")

#find_package(PCL)
# set(PCL_DIR "/home/helen/Program/PCL-1.6.0-Source/build")
find_package(PCL)

include_directories(${PCL_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS}  ${OPENNI_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS} ${EIGEN_LIBRARY_DIRS}  ${OPENNI_LIB_DIR})

# MESSAGE(STATUS "my sources include files direction: ${PCL_INCLUDE_DIRS}")
# MESSAGE(STATUS "my sources libs direction: ${PCL_LIBRARY_DIRS}")

set(VisPbMap_SRC ./src/main.cpp ./src/Plane.cpp ./src/Miscellaneous.cpp ./src/PbMap.cpp ./src/PbMapVisualizer.cpp)

# add_executable(VisPbMap ${VisPbMap_SRC})
rosbuild_add_executable(VisPbMap ${VisPbMap_SRC})
target_link_libraries(VisPbMap ${PCL_LIBRARIES}  ${OPENNI_LIBS})

rosbuild_add_library(visPbMap ${VisPbMap_SRC})
target_link_libraries(visPbMap ${PCL_LIBRARIES} ${OPENNI_LIBS})
