
cmake_minimum_required(VERSION 3.0.2)
project(neural_control)
set (CMAKE_CXX_STANDARD 11)
add_compile_options(-std=c++11)

set (WORKSPACE "../../../../../../..")
set (CONTROLLER "${WORKSPACE}/software/controllers/loft_alpha")
set (UTILS "${WORKSPACE}/software/utils")
set (SOFTWARE "${WORKSPACE}/software")
set (LOFT_CONTROLLER "${CONTROLLER}/neural_control")
set (ANN_FRAMEWORK "${UTILS}/ann-framework")
set (ANN_LIB "${UTILS}/ann-library")

message(${LOFT_CONTROLLER})
message(${UTILS})

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

include_directories("${catkin_INCLUDE_DIRS}" "${ANN_FRAMEWORK}" "${ANN_LIB}" "${SOFTWARE}" "${LOFT_CONTROLLER}")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(
	neural_control
	${UTILS}/interpolator2d
	${ANN_FRAMEWORK}/ann
	${ANN_FRAMEWORK}/neuron
	${ANN_FRAMEWORK}/synapse
	${LOFT_CONTROLLER}/neural_control
	${LOFT_CONTROLLER}/neural_unit
	${LOFT_CONTROLLER}/neural_controller
	${LOFT_CONTROLLER}/rosClass
	)

target_link_libraries(neural_control ${catkin_LIBRARIES})


add_dependencies(neural_control ${catkin_EXPORTED_TARGETS})
