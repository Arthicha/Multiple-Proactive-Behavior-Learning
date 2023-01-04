
cmake_minimum_required(VERSION 3.0.2)
project(robot_control)
set (CMAKE_CXX_STANDARD 11)
add_compile_options(-std=c++11)

set (WORKSPACE "../../../../../../..")
set (CONTROLLER "${WORKSPACE}/software/controllers/loft_alpha")
set (UTILS "${WORKSPACE}/software/utils")
set (SOFTWARE "${WORKSPACE}/software")
set (LOFT_CONTROLLER "${CONTROLLER}/robot_control")

message(${LOFT_CONTROLLER})
message(${UTILS})

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs)
include_directories("${catkin_INCLUDE_DIRS}" "${SOFTWARE}" "${LOFT_CONTROLLER}")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(
	robot_control
	${LOFT_CONTROLLER}/main_controller
	${LOFT_CONTROLLER}/rosClass
	)

target_link_libraries(robot_control ${catkin_LIBRARIES} SDL GL glut)


add_dependencies(robot_control ${catkin_EXPORTED_TARGETS})
