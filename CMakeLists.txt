cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package (Qt4)
find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)
find_package( OpenCV REQUIRED )

include (${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

QT4_WRAP_CPP(headers_MOC robot_gui.hh)

add_library(_robot_gui SHARED ${headers_MOC} robot_gui.cc 
#camera.cc
)
target_link_libraries(_robot_gui ${GAZEBO_LIBRARIES}
    ${QT_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
		${OpenCV_LIBS}
#		CameraPlugin
	)

add_library(_robot SHARED robot.cc)
target_link_libraries(_robot ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

set (KEYBOARD_DRIVE_SOURCES drive.cc autonomous.cc)
add_executable(drive ${KEYBOARD_DRIVE_SOURCES})
target_link_libraries(drive ${GAZEBO_LIBRARIES} ${OpenCV_LIBS} 
#CameraPlugin
)
