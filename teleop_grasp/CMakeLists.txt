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
		rospy
		roslib
		gazebo_ros
		ros_utils
		cv_camera
		franka_controllers
		franka_gripper
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
		rospy
		roslib
		gazebo_ros
		ros_utils
		cv_camera
		franka_controllers
		franka_gripper
		hand_tracking
	DEPENDS
		EIGEN3 # uppercase due to catkin_package() warning
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

add_executable(teleoperation src/teleoperation.cpp)
target_link_libraries(teleoperation ${catkin_LIBRARIES} teleop_grasp)

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

## header files
install(
	DIRECTORY
		include/teleop_grasp/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## gazebo stuff
install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## matplotlib style
add_custom_target(mplstyle_symlink ALL
	COMMAND ${CMAKE_COMMAND} -E make_directory "~/.config/matplotlib/stylelib"
	COMMAND ${CMAKE_COMMAND} -E create_symlink "${CMAKE_CURRENT_LIST_DIR}/config/teleop_grasp.mplstyle" "~/.config/matplotlib/stylelib/teleop_grasp.mplstyle"
)
# configure_file(config/teleop_grasp.mplstyle ~/.config/matplotlib/stylelib/teleop_grasp.mplstyle COPYONLY)
