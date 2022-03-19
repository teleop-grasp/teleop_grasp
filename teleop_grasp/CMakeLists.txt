cmake_minimum_required(VERSION 3.12)
project(teleop_grasp)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find catkin and any catkin packages
find_package(
	catkin REQUIRED
	COMPONENTS
		roscpp
		roslib
		ros_utils
		cv_camera
)

# find system libraries
find_package(Eigen3 REQUIRED)
# find_package(Boost REQUIRED)

# declare a catkin package
catkin_package(
	INCLUDE_DIRS
		include
		${EIGEN3_INCLUDE_DIR}
	LIBRARIES
		teleop_grasp
	CATKIN_DEPENDS
		roscpp
		roslib
		ros_utils
		cv_camera
	DEPENDS
		Eigen3
		# Boost
)

# additional locations of header files
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
	# ${Boost_INCLUDE_DIRS}
)
################################################################################################

# libraries
# REMEMBER TO ADD TO INSTALL TARGETS (bottom of file) !!!

## teleop_grasp
add_library(teleop_grasp src/teleop_grasp.cpp)
target_link_libraries(teleop_grasp ${catkin_LIBRARIES})

################################################################################################

# nodes (executables)

add_executable(demo_teleop_grasp examples/demo_teleop_grasp.cpp)
target_link_libraries(demo_teleop_grasp ${catkin_LIBRARIES} teleop_grasp)

add_executable(pipeline src/pipeline.cpp)
target_link_libraries(pipeline ${catkin_LIBRARIES} teleop_grasp)

################################################################################################

# install targets

## executables and/or libraries for installation
install(
	TARGETS
		teleop_grasp
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## install header files
install(
	DIRECTORY
		include/teleop_grasp/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

################################################################################################

# experiments (executables from /tests)

# message(STATUS "Addings experiments from 'tests/*/*.cpp'...")
# file(GLOB files CONFIGURE_DEPENDS "tests/*/*.cpp")
# foreach(file ${files})
# 	get_filename_component(name ${file} NAME_WE)
# 	message(STATUS "Found experiment: ${name}...")
# 	add_executable(${name} ${file})
# 	target_link_libraries(${name} ${catkin_LIBRARIES} teleop_grasp)
# endforeach()