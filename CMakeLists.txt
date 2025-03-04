cmake_minimum_required(VERSION 3.10)
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
  message_generation
  eigen_conversions
  dynamic_reconfigure
)

find_package(Eigen3 REQUIRED)
find_package(teaserpp REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)

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

# Generate messages in the 'msg' folder
 add_message_files(
   FILES
   gcodeAction.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

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
generate_dynamic_reconfigure_options(
  cfg/ReconfigureTF.cfg
  # cfg/DynReconf2.cfg
)

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
  INCLUDE_DIRS include
  #LIBRARIES seam_detection
  CATKIN_DEPENDS roscpp rospy std_msgs tf message_runtime
  DEPENDS  #system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Declare a C++ library
# these should be modules of the same library maybe...
add_library(cloudfilter
  src/cloudfilter.cpp
)

add_library(cloudutils
  src/cloudutils.cpp
)
target_link_libraries(cloudutils 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

add_library(cloudregistration
  src/cloudregistration.cpp
)
target_link_libraries(cloudregistration 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  teaserpp::teaser_registration
  teaserpp::teaser_io
  teaserpp::teaser_features
)

# test nodes for the libraries defined above, leave disables for compile speed
# # cloudutils_test (testing cloudutils.cpp/cloudutils.h)
# add_executable(cloudutils_test src/test/cloudutils_test.cpp)
# target_link_libraries(cloudutils_test 
#   cloudutils 
#   ${catkin_LIBRARIES} 
#   ${PCL_LIBRARIES}
# )

# # cloudfilter_test (testing cloudfilter.cpp/cloudfilter.h)
# add_executable(cloudfilter_test src/test/cloudfilter_test.cpp)
# target_link_libraries(cloudfilter_test 
#   cloudfilter 
#   ${catkin_LIBRARIES} 
#   ${PCL_LIBRARIES}
# )

# # cloudregisration_test (testing cloudregistration.cpp/cloudregistration.h)
# add_executable(cloudregistration_test src/test/cloudregistration_test.cpp)
# target_link_libraries(cloudregistration_test 
#   cloudregistration 
#   ${catkin_LIBRARIES} 
#   ${PCL_LIBRARIES}
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

###########################################
## nodes to be compiled with catkin_make ##
###########################################

##################################################
## primary nodes for the SEAM_DETECTION package ##
##################################################

# segment_clouds (class approach)
add_executable(segment_clouds src/segment_clouds.cpp)
target_link_libraries(segment_clouds 
  cloudfilter 
  cloudutils 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES} 
  Eigen3::Eigen 
)

# filter_dataset (class approach)
add_executable(filter_dataset src/filter_dataset.cpp)
target_link_libraries(filter_dataset 
  cloudfilter 
  cloudutils 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES} 
  Eigen3::Eigen 
)

## seam_detection
#add_executable(seam_detection src/seam_detection.cpp)
#target_link_libraries(seam_detection ${catkin_LIBRARIES} ${PCL_LIBRARIES} 
#  Eigen3::Eigen 
#  teaserpp::teaser_registration 
#  teaserpp::teaser_io
#  teaserpp::teaser_features
#)

# register_clouds - Registration Algorithm Testing Node(s) 
add_executable(register_clouds src/register_clouds.cpp)
target_link_libraries(register_clouds 
  cloudregistration 
  cloudfilter 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES} 
  Eigen3::Eigen 
  teaserpp::teaser_registration 
  teaserpp::teaser_io
  teaserpp::teaser_features
)


# seam_detection_RANSAC # this node needs updating
add_executable(seam_detection_RANSAC src/seam_detection_RANSAC.cpp)
target_link_libraries(seam_detection_RANSAC 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
)

######################################################
## Supporting nodes for the SEAM_DETECTION package  ##
## some may be commented out to save mem on compile ##
## uncomment to enable compile                      ##
######################################################

# filter_cloud
add_executable(filter_cloud src/filter_cloud.cpp)
target_link_libraries(filter_cloud 
  cloudfilter 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
)

# seam_detection_tf
add_executable(seam_detection_tf src/seam_detection_tf.cpp)
target_link_libraries(seam_detection_tf 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
)

# filter_cloud_tf
add_executable(filter_cloud_tf src/filter_cloud_tf.cpp)
target_link_libraries(filter_cloud_tf 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
)

# realsense_tf
#add_executable(realsense_tf src/realsense_tf.cpp)
#target_link_libraries(realsense_tf ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# reconfigure_server
add_executable(reconfigure_server src/reconfigure_server.cpp)
add_dependencies(reconfigure_server ${PROJECT_NAME}_gencfg)
target_link_libraries(reconfigure_server 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
)

# reconfigure_tf
add_executable(reconfigure_tf src/reconfigure_tf.cpp)
add_dependencies(reconfigure_tf ${PROJECT_NAME}_gencfg)
target_link_libraries(reconfigure_tf 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
)

# realsense_playback_tf
#add_executable(realsense_playback_tf src/realsense_playback_tf.cpp)
#target_link_libraries(realsense_playback_tf ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# realsense_playback_tf2
#add_executable(realsense_playback_tf2 src/realsense_playback_tf2.cpp)
#target_link_libraries(realsense_playback_tf2 ${catkin_LIBRARIES} ${PCL_LIBRARIES})
# rotate_cloud
#add_executable(rotate_cloud src/rotate_cloud.cpp)
#target_link_libraries(rotate_cloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# hue_segmentation
#add_executable(hue_segmentation src/hue_segmentation.cpp)
#target_link_libraries(hue_segmentation ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# crg_segmentation 
#add_executable(crg_segmentation src/crg_segmentation.cpp)
#target_link_libraries(crg_segmentation ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# cad_cloud
# add_executable(cad_cloud src/cad_cloud.cpp)
# target_link_libraries(cad_cloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# cad_cloud_bulk
# add_executable(cad_cloud_bulk src/cad_cloud_bulk.cpp)
# target_link_libraries(cad_cloud_bulk ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# save_rgbcloud
#add_executable(save_rgbcloud src/save_rgbcloud.cpp)
#target_link_libraries(save_rgbcloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})

###########################################################
## Aubo Robot system interface nodes                     ## 
## require aubo_description and aubo_control to compile  ##
## comment out these nodes if robot not present          ## 
###########################################################

# aubo_cmd
add_executable(aubo_cmd src/aubo_cmd.cpp)
target_link_libraries(aubo_cmd 
    ${catkin_LIBRARIES}
)

# get_cloud
#add_executable(get_cloud src/get_cloud.cpp)
#target_link_libraries(get_cloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# start_target_scan
#add_executable(start_target_scan src/start_target_scan.cpp)
#target_link_libraries(start_target_scan ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# get_target
#add_executable(get_target src/get_target.cpp)
#target_link_libraries(get_target ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# filter_target
#add_executable(filter_target src/filter_target.cpp)
#target_link_libraries(filter_target ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# start_source_scan
#add_executable(start_source_scan src/start_source_scan.cpp)
#target_link_libraries(start_source_scan ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# get_source
#add_executable(get_source src/get_source.cpp)
#target_link_libraries(get_source ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# filter_source
#add_executable(filter_source src/filter_source.cpp)
#target_link_libraries(filter_source ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# save_rgbcloud
add_executable(save_rgbcloud src/save_rgbcloud.cpp)
target_link_libraries(save_rgbcloud 
                      cloudfilter ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# teach_points
add_executable(teach_points src/teach_points.cpp)
target_link_libraries(teach_points ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# register_target_source - Registration and task execution with aubo arm
add_executable(register_target_source src/register_target_source.cpp)
target_link_libraries(register_target_source
  cloudutils
  cloudregistration
  cloudfilter
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES} 
  Eigen3::Eigen 
)

################################################################
## Teaser Nodes
################################################################
# Change this line to include your own executable file
# Link to teaserpp & Eigen3
# add_executable(teaser_cpp_ply src/teaser_cpp_ply.cc)
# target_link_libraries(teaser_cpp_ply Eigen3::Eigen teaserpp::teaser_registration teaserpp::teaser_io teaserpp::teaser_features)

# Copy the data files to build directory
# file(COPY ../example_data/
#        DESTINATION ./example_data/
#        FILES_MATCHING
#        PATTERN *.ply)

# add_executable(correspondence_grouping src/devel/correspondence_grouping.cpp)
# target_link_libraries(correspondence_grouping ${catkin_LIBRARIES} ${PCL_LIBRARIES})

##########################################################
## Nodes in developement for the SEAM_DETECTION package ##
##########################################################
# un-comment if you need to compile development nodes
# these have not been compiled successfully since they have been moved into /devel
# NOTE: move these back to src/ if they will not compile in src/devel

################################################################
## Archived nodes previously used with SEAM_DETECTION package ##
################################################################
# un-comment if you need to compile archived nodes
# these have not been compiled successfully since they have been moved into /archive
# NOTE: move these back to src/ if they will not compile in src/archive

# scan2cloud (from SZ)
#add_executable(scan2cloud src/scan2cloud.cpp)
#add_dependencies(scan2cloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) # from sz
#target_link_libraries(scan2cloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#install(TARGETS scan2cloud RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})           # from SZ

#add_executable(merge_cloud src/merge_cloud.cpp)
#add_dependencies(merge_cloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(merge_cloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#install(TARGETS merge_cloud RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})          # from SZ

#add_executable(publish_pcd src/publish_pcd.cpp)
#add_dependencies(publish_pcd ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) # from SZ
#target_link_libraries(publish_pcd ${catkin_LIBRARIES} ${PCL_LIBRARIES})

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
