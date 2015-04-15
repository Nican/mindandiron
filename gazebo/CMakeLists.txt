cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

ADD_DEFINITIONS(-std=c++11 -Wall)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
#find_package(Qt5Widgets)
#find_package(Qt5OpenGL)
find_package(PCL)
find_package(OpenCV)
#find_package(apriltags REQUIRED)

include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
link_directories    (lib)
add_definitions     (${PCL_DEFINITIONS})

#message(STATUS "PCL_DEFINITIONS=${PCL_DEFINITIONS}")
#message(STATUS "PCL_LIBRARY_DIRS=${PCL_LIBRARY_DIRS}")
#message(STATUS "PCL_INCLUDE_DIRS=${PCL_INCLUDE_DIRS}")
#message(STATUS "PCL_LIBRARIES=${PCL_LIBRARIES}")

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(/usr/local/include/libfreenect2/tinythread/)

link_directories(${Boost_LIBRARY_DIRS})
link_directories(lib)


add_library(kratos_base SHARED trajectory.cpp pointcloud.cpp odometry.cpp)
target_link_libraries(kratos_base apriltags ${Boost_LIBRARIES} ${OpenCV_LIBS}  ${PCL_LIBRARIES} zmq msgpack Box2D)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(kratos SHARED gazebo2.cpp)
target_link_libraries(kratos ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} kratos_base)

add_executable(park_viewer park_viewer.cpp)
target_link_libraries(park_viewer ${Boost_LIBRARIES} kratos_base)

add_subdirectory(telemetry)
add_subdirectory(robot)
