 cmake_minimum_required(VERSION 3.10)
#cmake_minimum_required(VERSION 2.8.3)
#cmake_minimum_required(VERSION 3.0.2)
#cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(seam_detection)

set (CMAKE_CXX_STANDARD 14)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  laser_geometry
  sensor_msgs
  pcl_conversions
  pcl_ros
  sensor_msgs
)

find_package(Eigen3 REQUIRED)


find_package(teaserpp REQUIRED)
find_package(Boost 1.58 REQUIRED)
find_package(PCL 1.10 REQUIRED) # COMPONENTS common features kdtree)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES seam_detection
#  CATKIN_DEPENDS roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/seam_detection.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/seam_detection_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_seam_detection.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)



include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

###########################################
## nodes to be compiled with catkin_make ##
###########################################

###############################################
## primary nodes for the SEAM_DETECTION package
###############################################

# seam_detection
add_executable(seam_detection src/seam_detection.cpp)
target_link_libraries(seam_detection ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# seam_detection_ICP
add_executable(seam_detection_ICP src/seam_detection_ICP.cpp)
target_link_libraries(seam_detection_ICP ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# seam_detection_RANSAC
add_executable(seam_detection_RANSAC src/seam_detection_RANSAC.cpp)
target_link_libraries(seam_detection_RANSAC ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#####################################################
## Supporting nodes for the SEAM_DETECTION package ##
#####################################################

# seam_detection_tf
add_executable(seam_detection_tf src/seam_detection_tf.cpp)
target_link_libraries(seam_detection_tf ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# cad_cloud
add_executable(cad_cloud src/cad_cloud.cpp)
target_link_libraries(cad_cloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# cad_cloud_bulk
add_executable(cad_cloud_bulk src/cad_cloud_bulk.cpp)
target_link_libraries(cad_cloud_bulk ${catkin_LIBRARIES} ${PCL_LIBRARIES})

##########################################################
## Nodes in developement for the SEAM_DETECTION package ##
##########################################################
# un-comment if you need to compile development nodes
# these have not been compiled successfully since they have been moved into /devel
# NOTE: move these back to src/ if they will not compile in src/devel

add_executable(seam_detection_teaser src/seam_detection_teaser.cpp)
target_link_libraries(seam_detection_teaser 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES} 
  Eigen3::Eigen 
  teaserpp::teaser_registration 
  teaserpp::teaser_io
)


#######################################################
## Registration Examples - Algorithm Testing Node(s) ##
#######################################################

add_executable(registration_examples src/registration_examples.cpp)
target_link_libraries(registration_examples 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES} 
  Eigen3::Eigen 
  teaserpp::teaser_registration 
  teaserpp::teaser_io
)




# Teaser Stuff is all disabled for now
# Change this line to include your own executable file
# Link to teaserpp & Eigen3
# add_executable(teaser_cpp_ply src/teaser_cpp_ply.cc)
# target_link_libraries(teaser_cpp_ply Eigen3::Eigen teaserpp::teaser_registration teaserpp::teaser_io)

# Change this line to include your own executable file
#add_executable(seam_detection_teaser src/seam_detection_teaser.cpp)
# Link to teaserpp & Eigen3 // teaser::features requires PCL 1.9
#target_link_libraries(seam_detection_teaser Eigen3::Eigen teaserpp::teaser_registration teaserpp::teaser_features teaserpp::teaser_io #${PCL_LIBRARIES})

# Copy the data files to build directory
# file(COPY ../example_data/
#        DESTINATION ./example_data/
#        FILES_MATCHING
#        PATTERN *.ply)

# add_executable(correspondence_grouping src/devel/correspondence_grouping.cpp)
# target_link_libraries(correspondence_grouping ${catkin_LIBRARIES} ${PCL_LIBRARIES})

################################################################
## Archived nodes previously used with SEAM_DETECTION package ##
################################################################
# un-comment if you need to compile archived nodes
# these have not been compiled successfully since they have been moved into /archive
# NOTE: move these back to src/ if they will not compile in src/archive

# import_seam
# add_executable(import_seam src/archive/import_seam.cpp)
# target_link_libraries(import_seam ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# filter_cloud
# add_executable(filter_cloud src/archive/filter_cloud.cpp)
# target_link_libraries(filter_cloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# plane_filter
# add_executable(plane_filter_RANSAC src/archive/plane_filter_RANSAC.cpp)
# target_link_libraries(plane_filter_RANSAC ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# ransac_plane
# add_executable(ransac_plane src/archive/ransac_plane.cpp)
# target_link_libraries(ransac_plane ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# segment_plane
# add_executable(segment_plane src/archive/segment_plane.cpp)
# target_link_libraries(segment_plane ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# segment_cylinder
# add_executable(segment_cylinder src/archive/segment_cylinder.cpp)
# target_link_libraries(segment_cylinder ${catkin_LIBRARIES} ${PCL_LIBRARIES})