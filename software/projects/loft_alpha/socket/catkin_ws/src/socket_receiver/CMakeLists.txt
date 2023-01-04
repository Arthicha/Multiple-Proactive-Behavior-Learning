
cmake_minimum_required(VERSION 3.0.2)
project(socket_receiver)
set (CMAKE_CXX_STANDARD 11)
add_compile_options(-std=c++11)

set (WORKSPACE "../../../../../../..")
set (CONTROLLER "${WORKSPACE}/software/controllers/loft_alpha")
set (UTILS "${WORKSPACE}/software/utils")
set (SOFTWARE "${WORKSPACE}/software")
set (LOFT_CONTROLLER "${CONTROLLER}/socket")
set (JOYSTICK_LIBS "${WORKSPACE}/software/utils/socket")

catkin_install_python(PROGRAMS receiver.py
  DESTINATION bin
)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs nav_2d_msgs)
include_directories("${catkin_INCLUDE_DIRS}" "${SOFTWARE}" "${LOFT_CONTROLLER}")



