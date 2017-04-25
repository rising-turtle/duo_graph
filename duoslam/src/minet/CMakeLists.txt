
add_library(minet SHARED ClientNet.h Com.h ExternalCom.h ClientNet.cpp Com.cpp ExternalCom.cpp)

add_executable(client ClientMain.cpp )
target_link_libraries (client minet pthread)

add_executable(server ServerMain.cpp )
target_link_libraries (server minet pthread)

