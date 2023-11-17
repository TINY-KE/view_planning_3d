cmake_minimum_required(VERSION 2.8.3)
project(view_planning)

SET(CMAKE_BUILD_TYPE Debug)  #zhjd

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  diagnostic_msgs
  geometry_msgs
  manipulation_msgs
  moveit_msgs
  moveit_ros_perception
  moveit_ros_planning_interface
  roscpp
  rospy
  sensor_msgs
  shape_msgs
  std_msgs
  std_srvs
  tf
  trajectory_msgs
  visualization_msgs
  rviz_visual_tools
  moveit_visual_tools
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 3.1.0 REQUIRED)


# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)   # TODO: automatic search libs

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

find_package(gpmp2 REQUIRED)
include_directories(${gpmp2_INCLUDE_DIR})


set(OpenCV_DIR "/home/zhjd/thirdparty/opencv3.4.10/opencv-3.4.10/build/")
find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()
set(cv_bridge_DIR "/home/zhjd/thirdparty/vision_opencv-indigo/cv_bridge/cmake")

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
#   control_msgs#   diagnostic_msgs#   geometry_msgs#   manipulation_msgs#   moveit_msgs#   moveit_msgs#   sensor_msgs#   shape_msgs#   std_msgs#   trajectory_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
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
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES marm_planning
#  CATKIN_DEPENDS actionlib control_msgs diagnostic_msgs geometry_msgs manipulation_msgs moveit_msgs moveit_msgs roscpp rospy sensor_msgs shape_msgs std_msgs std_srvs tf trajectory_msgs
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
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/marm_planning.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/marm_planning_node.cpp)

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

# add_executable(test_random_node src/test_random.cpp)
# target_link_libraries(test_random_node ${catkin_LIBRARIES})

# add_executable(test_custom_node src/test_custom.cpp)
# target_link_libraries(test_custom_node ${catkin_LIBRARIES})

# add_executable(add_collision_objct_node src/add_collision_objct.cpp)
# target_link_libraries(add_collision_objct_node ${catkin_LIBRARIES})

# add_executable(remove_collision_objct_node src/remove_collision_objct.cpp)
# target_link_libraries(remove_collision_objct_node ${catkin_LIBRARIES})

# add_executable(check_collision_node src/check_collision.cpp)
# target_link_libraries(check_collision_node ${catkin_LIBRARIES})

# add_executable(test_cartesian_path_node src/test_cartesian_path.cpp)
# target_link_libraries(test_cartesian_path_node ${catkin_LIBRARIES} ${EIGEN3_LIBS})

# add_executable(fabo_cartesian_path_node src/fabo_cartesian_path.cpp)
# target_link_libraries(fabo_cartesian_path_node ${catkin_LIBRARIES} ${EIGEN3_LIBS})

# add_executable(fabo_ik_tf   src/fabo_ik_tf.cpp)
# target_link_libraries(fabo_ik_tf ${catkin_LIBRARIES} ${EIGEN3_LIBS})


# add_executable(fabo_cartesian_path_one src/fabo_cartesian_path_one.cpp)
# target_link_libraries(fabo_cartesian_path_one ${catkin_LIBRARIES} ${EIGEN3_LIBS})

# add_executable(generate_candidate src/generate_candidate.cpp )
# src/Converter.cc
# target_link_libraries(gen5erate_candidate ${catkin_LIBRARIES} ${EIGEN3_LIBS} ${OpenCV_LIBS})

# add_executable(cartesian_candidate src/cartesian_candidate.cpp )
# target_link_libraries(cartesian_candidate ${catkin_LIBRARIES} ${EIGEN3_LIBS} ${OpenCV_LIBS})

# add_executable(candidate_ik_fabo src/candidate_ik_fabo.cpp  )
# target_link_libraries(candidate_ik_fabo ${cEIGEN3_LIBST} gpmp2)


# message(STATUS "Fabo Arm Test")
# add_executable(fabo_test src/test.cpp)
# target_link_libraries(fabo_test   ${catkin_LIBRARIES} ${EIGEN3_LIBS} ${GTSAM_LIBRARIES} gpmp2 )  #${Boost_LIBRARIES}

message(STATUS "Arm7 gpmp Test")
add_executable(gpmp_realman src_gpmp/gpmp_realman.cpp)
target_link_libraries(gpmp_realman   ${catkin_LIBRARIES} ${EIGEN3_LIBS} ${GTSAM_LIBRARIES} gpmp2 ${OpenCV_LIBS})  #${Boost_LIBRARIES}



message(STATUS "Arm7 gpmp Test")
add_executable(gpmp_realman_debug src_gpmp/gpmp_realman_setfromik_debug.cpp)
target_link_libraries(gpmp_realman_debug   ${catkin_LIBRARIES} ${EIGEN3_LIBS} ${GTSAM_LIBRARIES} gpmp2 ${OpenCV_LIBS})  #${Boost_LIBRARIES}

# add_executable(sdf_visulization src_gpmp/sdf_visulization.cpp)
# target_link_libraries(sdf_visulization   ${catkin_LIBRARIES} ${EIGEN3_LIBS} ${GTSAM_LIBRARIES} gpmp2 )  #${Boost_LIBRARIES}


# add_executable(test_bwdist src/test_bwdist.cpp)
# target_link_libraries(test_bwdist   ${catkin_LIBRARIES} ${EIGEN3_LIBS} ${OpenCV_LIBS})  #${Boost_LIBRARIES}



# add_executable(test_dist src/test_dis.cpp)
# target_link_libraries(test_dist   ${catkin_LIBRARIES} ${EIGEN3_LIBS} ${OpenCV_LIBS})




# add_executable(robot_state_ptr src_moveit/robot_state_ptr.cpp)
# target_link_libraries(robot_state_ptr   ${catkin_LIBRARIES} ${EIGEN3_LIBS} )

add_executable(robot_state_ptr2 src_moveit/robot_state_ptr2.cpp)
target_link_libraries(robot_state_ptr2   ${catkin_LIBRARIES} ${EIGEN3_LIBS} )