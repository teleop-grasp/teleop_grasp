cmake_minimum_required(VERSION 3.4)
project(franka_controllers)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find system libraries
find_package(Eigen3 REQUIRED)

find_package(
	catkin REQUIRED
	COMPONENTS
		roscpp
		roslib
		std_msgs
		ros_utils
		controller_manager
		controller_interface
		pluginlib
		franka_hw
)

catkin_package(
	INCLUDE_DIRS
		include
		${EIGEN3_INCLUDE_DIR}
	LIBRARIES
		franka_controllers
	CATKIN_DEPENDS
		roscpp
		roslib
		std_msgs
		ros_utils
		controller_manager
		controller_interface
		pluginlib
		franka_hw
	DEPENDS
		EIGEN3
)

# find_package: CATKIN_DEPENDS: franka_ws -> ${catkin_INCLUDE_DIRS} & ${catkin_LIBRARIES}
# catkin_package: generates catkin build files for "catkin build"
# 
# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
)

################################################################################################

# libraries
# REMEMBER TO ADD TO INSTALL TARGETS (bottom of file) !!!

## franka_controller
add_library(franka_controllers
		# src/franka_controllers.cpp # interface
		src/joint_pd_gravity_controller.cpp
		src/cartesian_pd_nullspace_controller.cpp
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

