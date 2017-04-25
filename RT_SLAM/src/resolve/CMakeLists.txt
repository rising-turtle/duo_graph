
set(SRCS SubmapReader.cpp submap_reader.cpp)
rosbuild_add_executable(submap_reader ${SRCS})
target_link_libraries(submap_reader ${OpenCV_LIBRARIES})


