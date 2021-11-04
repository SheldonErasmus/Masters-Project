cmake_minimum_required(VERSION 3.0.2)
project(controller)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "-std=c++17")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(gazebo_DIR "/usr/lib/x86_64-linux-gnu/cmake/gazebo")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_ros
  gazebo REQUIRED
)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  ${Boost_INCLUDE_DIR} 
  ${catkin_INCLUDE_DIRS} 
  ${GAZEBO_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
#   ${GAZEBO_INCLUDE_DIRS}
)

#catkin_install_python(PROGRAMS
#  src/MPC.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

add_library(${PROJECT_NAME} src/control.cpp)
target_link_libraries(
  ${PROJECT_NAME} 
  ${catkin_LIBRARIES} 
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES})
