
cmake_minimum_required(VERSION 3.0.2)
project(joystick)
set (CMAKE_CXX_STANDARD 11)
add_compile_options(-std=c++11)

set (WORKSPACE "../../../../../../..")
set (CONTROLLER "${WORKSPACE}/software/controllers/loft_alpha")
set (UTILS "${WORKSPACE}/software/utils")
set (SOFTWARE "${WORKSPACE}/software")
set (LOFT_CONTROLLER "${CONTROLLER}/joystick")
set (JOYSTICK_LIBS "${WORKSPACE}/software/utils/joystick")

message(${LOFT_CONTROLLER})
message(${UTILS})

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)
include_directories("${catkin_INCLUDE_DIRS}" "${SOFTWARE}" "${LOFT_CONTROLLER}")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(
	joystick
	${LOFT_CONTROLLER}/joystick_teleop
	${JOYSTICK_LIBS}/joystick
	)

target_link_libraries(joystick ${catkin_LIBRARIES} SDL GL glut)


add_dependencies(joystick ${catkin_EXPORTED_TARGETS})
