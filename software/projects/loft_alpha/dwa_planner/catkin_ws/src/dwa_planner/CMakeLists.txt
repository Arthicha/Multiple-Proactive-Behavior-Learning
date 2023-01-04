
cmake_minimum_required(VERSION 3.0.2)
project(dwa_planner)
set (CMAKE_CXX_STANDARD 11)
add_compile_options(-std=c++11)

set (WORKSPACE "../../../../../../..")
set (CONTROLLER "${WORKSPACE}/software/controllers/loft_alpha")
set (UTILS "${WORKSPACE}/software/utils")
set (SOFTWARE "${WORKSPACE}/software")
set (LOFT_CONTROLLER "${CONTROLLER}/dwa_planner")
set (ANN_FRAMEWORK "${UTILS}/ann-framework")
set (ANN_LIB "${UTILS}/ann-library")

message(${LOFT_CONTROLLER})
message(${UTILS})

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs nav_2d_msgs stereo_msgs)

include_directories("${catkin_INCLUDE_DIRS}" "${ANN_FRAMEWORK}" "${ANN_LIB}" "${SOFTWARE}" "${LOFT_CONTROLLER}")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(
	dwa_planner
	${LOFT_CONTROLLER}/demo
	${LOFT_CONTROLLER}/planner
	${LOFT_CONTROLLER}/dwa
	${LOFT_CONTROLLER}/rosClass
	)

target_link_libraries(dwa_planner ${catkin_LIBRARIES} SDL GL glut)


add_dependencies(dwa_planner ${catkin_EXPORTED_TARGETS})
