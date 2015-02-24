cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

ADD_DEFINITIONS(-std=c++11)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(Qt4 REQUIRED)
find_package (VTK REQUIRED)
#find_package(Qt5Widgets)
#find_package(Qt5OpenGL)
find_package(PCL 1.8.0 EXACT REQUIRED)
find_package(OpenCV)
#find_package(apriltags REQUIRED)
#find_package(osg)

include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

message(STATUS "PCL_DEFINITIONS=${PCL_DEFINITIONS}")
message(STATUS "PCL_LIBRARY_DIRS=${PCL_LIBRARY_DIRS}")
message(STATUS "PCL_INCLUDE_DIRS=${PCL_INCLUDE_DIRS}")
message(STATUS "PCL_LIBRARIES=${PCL_LIBRARIES}")

set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

#include (FindPkgConfig)
#if (PKG_CONFIG_FOUND)
#  pkg_check_modules(GAZEBO gazebo)
#endif()
#include_directories(${GAZEBO_INCLUDE_DIRS})
#link_directories(${GAZEBO_LIBRARY_DIRS})

#message(${GAZEBO_LIBRARY_DIRS})

add_library(kratos_base SHARED robot.cc trajectory.cpp state.cc pointcloud.cpp april.cpp)
target_link_libraries(kratos_base apriltags ${Boost_LIBRARIES} ${OpenCV_LIBS}  ${PCL_LIBRARIES} zmq msgpack Box2D)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(kratos SHARED gazebo.cc)
target_link_libraries(kratos ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} kratos_base)

#add_library(kratos_camera SHARED camera.cpp)
#target_link_libraries(kratos_camera ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} DepthCameraPlugin)




# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

#QT5_WRAP_CPP(kratos_telemetry telemetry.h)
ADD_EXECUTABLE(kratos_telemetry telemetry/telemetry.cpp telemetry/pointcloud.cpp)
#osg osgQt osgWidget osgViewer osgUtil osgGA osgDB
TARGET_LINK_LIBRARIES(kratos_telemetry ${QT_LIBRARIES}  kratos_base ${VTK_LIBRARIES}) #Qt5::Widgets Qt5::OpenGL

#ADD_EXECUTABLE(box2dtest box2dtest.cpp)
#TARGET_LINK_LIBRARIES(box2dtest Box2D)