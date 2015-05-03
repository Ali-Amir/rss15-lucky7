##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rss_msgs)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED rosjava_build_tools gencpp genmsg message_generation)

add_message_files(
  DIRECTORY msg
  FILES
  AnalogStatusMsg.msg
  ArmMsg.msg
  BreakBeamMsg.msg
  BumpMsg.msg
  DigitalStatusMsg.msg
  EncoderMsg.msg
  MotionMsg.msg
  OdometryMsg.msg
  ResetMsg.msg
  SonarMsg.msg
  RobotLocation.msg
)

add_service_files(
  DIRECTORY msg
  FILES
  LocFree.srv
)

generate_messages()

# Set the gradle targets you want catkin's make to run by default, e.g.
#   catkin_rosjava_setup(installApp)
# Note that the catkin_create_rosjava_xxx scripts will usually automatically
# add tasks to this for you when you create subprojects.
catkin_rosjava_setup()

catkin_package(
  CATKIN_DEPENDS message_runtime
)

##############################################################################
# Installation
##############################################################################

# Change this to match the maven group name you have specified in the 
# allprojects closure the root build.gradle
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/com/github/rosjava/${PROJECT_NAME}/ 
        DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/com/github/rosjava/${PROJECT_NAME})
