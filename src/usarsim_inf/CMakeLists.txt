cmake_minimum_required(VERSION 2.8.3)
project(usarsim_inf)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  actionlib
  control_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   ToolchangerStatus.msg
#   Message2.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   control_msgs
   geometry_msgs
   nav_msgs
   sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES usarsim_inf
  CATKIN_DEPENDS actionlib control_msgs geometry_msgs nav_msgs sensor_msgs tf message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  msg_gen/cpp/include
)

## Declare a C++ library
add_library(usarsim_inf
   src/ulapi.cpp
   src/genericInf.cpp
   src/servoInf.cpp
   src/usarsimInf.cpp
   src/usarsimMisc.cpp
   src/simware.cpp
 )

## Declare a C++ executable
add_executable(usarsim_node src/usarsim.cpp)
add_executable(usarsim_urdf src/usarsim_urdf_gen.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(usarsim_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(usarsim_node usarsim_inf ${catkin_LIBRARIES})
target_link_libraries(usarsim_urdf usarsim_inf ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
#install(TARGETS usarsim_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

