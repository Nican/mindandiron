

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

find_package(Qt5Widgets)
find_package(Qt5SerialPort)

add_executable(kratos_robot realrobot.cpp teensey.cpp)
target_link_libraries(kratos_robot Qt5::Widgets Qt5::SerialPort freenect2 kratos_base)