#!/bin/bash

# project meta-data

WS_NAME="teleop_ws"
WS_PATH="$PWD/$WS_NAME"
PKG_NAME="teleop_grasp"
PROJ_NAME="teleop-grasp"
GIT_URI="git@github.com:teleop-grasp/teleop_grasp.git" # must be SSH
GIT_BRANCH="main"
DEMO_CMD="roslaunch teleop_grasp workcell.launch"

# check that script is sourced
# https://stackoverflow.com/questions/2683279/how-to-detect-if-a-script-is-being-sourced
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]
then
	echo "Must be sourced; please run 'source setup.bash'."; exit
fi

# check dependencies
if ! hash rosdep vcs &> /dev/null
then
	echo "Unmet dependecies. Please make sure that rosdep and vsctool are installed."
	return
	exit
fi

# install apt dependencies (vcstool and rosdep)
# sudo apt install git python3-vcstool python3-rosdep -y

# confirm installation
read -p "This script will create the catkin workspace at '$WS_PATH' and install $PROJ_NAME. Continue? [Y/n] " -n 1 -r
echo
if [[ $REPLY =~ ^[Nn]$ ]]; then return;exit; fi

# setup workspace
echo -e  "\n\e[104mCreating catkin workspace...\e[49m\n" && sleep 1
mkdir -p $WS_PATH/src
cd $WS_PATH
catkin init

# determine URL of .repos file from GIT_URI using regex (yikes)
GIT_REPO=$(echo $GIT_URI | grep -Po "(?<=:)(.*)(?=.git)") # extract the "user/repo" part
REPOS_FILE_PATH="https://raw.githubusercontent.com/$GIT_REPO/$GIT_BRANCH/.repos"

# for privates repos (deprecated)
# echo -e  "\n\e[104mCloning project package...\e[49m\n" && sleep 1
# git clone $GIT_URI $WS_PATH/src/$PKG_NAME
# REPOS_FILE_PATH=$WS_PATH/src/$PKG_NAME/.

# clone packages (vcstool)
# echo -e  "\n\e[104mCloning packages...\e[49m\n" && sleep 1
echo -e  "\n\e[104mCloning packages from .repos file...\e[49m\n" && sleep 1
cd $WS_PATH
vcs import < $REPOS_FILE_PATH
vcs pull src

# update and install any system dependencies (rosdep)
echo -e  "\n\e[104mInstalling rosdep dependencies...\e[49m\n" && sleep 1
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# build the workspace
echo -e  "\n\e[104mBuilding workspace...\e[49m\n" && sleep 1
catkin build

# add alias to .bashrc
echo -e "\n# $PROJ_NAME\nalias $WS_NAME='source $WS_PATH/devel/setup.bash && cd $WS_PATH'" >> ~/.bashrc
source ~/.bashrc

# delete script
cd .. && rm setup.bash

# finish
echo -e  "\n\e[104mInstallation complete!\e[49m\n"
echo -e "The project has been installed.\n\nSource the workspace and run '$DEMO_CMD' to verify the installation. Run '$WS_NAME' in any terminal to automatically configure the workspace and navigate to its directory.\n"
