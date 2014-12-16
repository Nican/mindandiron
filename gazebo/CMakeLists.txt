cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

ADD_DEFINITIONS(-std=c++11)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(Qt5Widgets)
find_package(Qt5OpenGL)
#find_package(osg)


include_directories(${Boost_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(kratos_base SHARED robot.cc trajectory.cpp state.cc)
target_link_libraries(kratos_base ${Boost_LIBRARIES} zmq msgpack Box2D)


add_library(kratos SHARED kratos.cc)
target_link_libraries(kratos ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES}  kratos_base)

add_library(kratos_camera SHARED camera.cpp)
target_link_libraries(kratos_camera ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} DepthCameraPlugin)




# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

#INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

#QT5_WRAP_CPP(kratos_telemetry telemetry.h)
ADD_EXECUTABLE(kratos_telemetry telemetry.cpp)
#osg osgQt osgWidget osgViewer osgUtil osgGA osgDB
TARGET_LINK_LIBRARIES(kratos_telemetry Qt5::Widgets Qt5::OpenGL kratos_base) 

#ADD_EXECUTABLE(box2dtest box2dtest.cpp)
#TARGET_LINK_LIBRARIES(box2dtest Box2D)