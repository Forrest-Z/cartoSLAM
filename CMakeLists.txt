cmake_minimum_required(VERSION 2.8.12)  # Ships with Ubuntu 14.04 (Trusty)

project(carto_slam)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++11 -Wall  -O0 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall   -O0 -march=native")


  
find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(Ceres REQUIRED COMPONENTS SparseLinearAlgebraLibrary)
find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(PCL 1.2 REQUIRED)
#find_package(Lua REQUIRED)
set(LUA_INCLUDE_DIR "/usr/local/include/")
set(LUA_LIBRARIES "/usr/local/lib/liblua.a")

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs  message_filters laser_geometry  rosbag)



#catkin_package(
#    INCLUDE_DIRS
#    CATKIN_DEPENDS roscpp nav_msgs  message_filters laser_geometry  rosbag
#    DEPENDS EIGEN3
#  )

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PROJECT_BINARY_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${LUA_INCLUDE_DIR})
include_directories(${CERES_INCLUDE_DIRS})
file(GLOB_RECURSE ALL_SRCS "*.cc")
file(GLOB_RECURSE ALL_EXECUTABLES "*main.cc")

list(REMOVE_ITEM ALL_SRCS ${ALL_EXECUTABLES})


file(GLOB_RECURSE ALL_PROTOS "*.proto")
set(ALL_PROTO_SRCS)
set(ALL_PROTO_HDRS)
foreach(ABS_FIL ${ALL_PROTOS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

  list(APPEND ALL_PROTO_SRCS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
  list(APPEND ALL_PROTO_HDRS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")

  add_custom_command(
    OUTPUT "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc"
           "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h"
    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --cpp_out  ${PROJECT_BINARY_DIR} -I
      ${PROJECT_SOURCE_DIR} ${ABS_FIL}
    DEPENDS ${ABS_FIL}
    COMMENT "Running C++ protocol buffer compiler on ${ABS_FIL}"
    VERBATIM
  )
endforeach()

#message(STATUS  ${catkin_LIBRARIES} )
#message(STATUS  
#${catkin_LIBRARIES}
#${Boost_LIBRARIES}
#${PROTOBUF_LIBRARY}
#)

#message(STATUS  ${ALL_SRCS} )

add_executable(${PROJECT_NAME}  ${ALL_SRCS} ${ALL_EXECUTABLES} ${ALL_PROTO_SRCS})

add_definitions(${PCL_DEFINITIONS})

#message(FATAL_ERROR ${LUA_LIBRARIES})

target_link_libraries(${PROJECT_NAME}
${LUA_LIBRARIES}
${catkin_LIBRARIES}
${Boost_LIBRARIES}
${PROTOBUF_LIBRARY}
${CERES_LIBRARIES}
glog
dl
)


#${ALL_SRCS}