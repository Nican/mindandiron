

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

find_package(Qt5Widgets)
find_package(Qt5SerialPort)
find_package(Qt5Sql)
find_package(Qt5Concurrent)
#find_package(Qt5Multimedia)

add_executable(kratos_robot 
	realrobot.cpp 
	gazeborobot.cpp
	teensey.cpp 
	../nzmqt/nzmqt.cpp 
	main.cpp
	robot.cpp
	trajectory2.cpp
	state.cpp
	camera.cpp
	../april.cpp
)

target_link_libraries(kratos_robot Qt5::Widgets Qt5::SerialPort Qt5::Sql Qt5::Concurrent freenect2 kratos_base)