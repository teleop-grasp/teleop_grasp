cmake_minimum_required(VERSION 3.4)
project(franka_controllers)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS -fno-fast-math)
add_compile_options(-Wall -Wextra -pedantic)

# release mode (unless otherwise specified)
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif()

# find system libraries
find_package(Eigen3 REQUIRED)

find_package(
	catkin REQUIRED
	COMPONENTS
		roscpp
		roslib
		ros_utils
		controller_manager
		controller_interface
		pluginlib
		realtime_tools
		franka_hw
		# plotjuggler
		eigen_conversions
		std_msgs
		geometry_msgs
		message_generation
		message_runtime
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
)

################################################################################################

# messages and services

add_message_files(
	FILES
		CartesianAdmittanceControllerDebug.msg
)

generate_messages(
	DEPENDENCIES
		std_msgs
		geometry_msgs
)

################################################################################################

# define catkin package

catkin_package(
	INCLUDE_DIRS
		include
		${EIGEN3_INCLUDE_DIR}
	LIBRARIES
		franka_controllers
	CATKIN_DEPENDS
		roscpp
		roslib
		ros_utils
		controller_manager
		controller_interface
		pluginlib
		realtime_tools
		franka_hw
		plotjuggler
		eigen_conversions
		std_msgs
		geometry_msgs
		message_generation
		message_runtime
	DEPENDS
		EIGEN3
)

################################################################################################

# libraries
# REMEMBER TO ADD TO INSTALL TARGETS (bottom of file) !!!

## franka_controllers
add_library(franka_controllers
		# src/franka_controllers.cpp # interface
		src/joint_pd_gravity_controller.cpp
		src/cartesian_pd_nullspace_controller.cpp
		src/cartesian_admittance_controller.cpp
)
target_link_libraries(franka_controllers ${catkin_LIBRARIES})

################################################################################################

# nodes

# add_executable(demo_franka_controllers examples/demo_franka_controllers.cpp)
# target_link_libraries(demo_franka_controllers ${catkin_LIBRARIES} franka_controllers)

################################################################################################

# install targets

## executables and/or libraries for installation
install(
	TARGETS
		franka_controllers
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)


## install header files
install(
	DIRECTORY
		include/franka_controllers/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## install ros_control plugin
install(FILES franka_controllers_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})