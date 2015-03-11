find_package(Qt4 REQUIRED)
find_package (VTK REQUIRED)

INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

#QT5_WRAP_CPP(kratos_telemetry telemetry.h)
ADD_EXECUTABLE(kratos_telemetry telemetry.cpp pointcloud.cpp)
#osg osgQt osgWidget osgViewer osgUtil osgGA osgDB
TARGET_LINK_LIBRARIES(kratos_telemetry ${QT_LIBRARIES}  kratos_base ${VTK_LIBRARIES}) #Qt5::Widgets Qt5::OpenGL

