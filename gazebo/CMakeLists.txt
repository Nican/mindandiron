cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

ADD_DEFINITIONS(-std=c++11)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(kratos SHARED kratos.cc robot.cc)
target_link_libraries(kratos ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})