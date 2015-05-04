##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(challenge)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED COMPONENTS
  rosjava_build_tools
  message_generation
  roslaunch
)

roslaunch_add_file_check(launch)

# Set the gradle targets you want catkin's make to run by default, e.g.
#   catkin_rosjava_setup(installApp)
# Note that the catkin_create_rosjava_xxx scripts will usually automatically
# add tasks to this for you when you create subprojects.
catkin_rosjava_setup(installApp publishMavenJavaPublicationToMavenRepository)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

#add_message_files(
#  DIRECTORY
#  ../rss_msgs/msg
#  FILES
#  MotionMsg.msg
#  BumpMsg.msg
#)

##############################################################################
# Installation
##############################################################################

# Change this to match the maven group name you have specified in the 
# allprojects closure the root build.gradle
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/com/github/rosjava/${PROJECT_NAME}/ 
        DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/com/github/rosjava/${PROJECT_NAME})
