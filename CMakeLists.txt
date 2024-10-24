cmake_minimum_required(VERSION 2.8.3)
project(view_planning)

# 各类库
set(OpenCV_DIR "/home/robotlab/thirdparty/opencv-3.4.10/build")
set(cv_bridge_DIR "/home/zhjd/thirdparty/vision_opencv-indigo/cv_bridge/cmake")


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
        # manipulation_msgs   #used for ubuntu18
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
        gazebo_msgs
        tf2_geometry_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(gazebo REQUIRED)


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

include_directories(
        ${GAZEBO_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
endif()

# 包含 OpenCV 头文件路径
include_directories(${OpenCV_INCLUDE_DIRS})

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
        # INCLUDE_DIRS include
        # LIBRARIES connect_gazebo
        # CATKIN_DEPENDS roscpp std_msgs  gazebo_msgs
        # DEPENDS system_lib
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
        #  /home/robotlab/QSP-SLAM-ellipsoid-associdate
        # ${PROJECT_SOURCE_DIR}/Thirdparty/g2o
)


# 一、LZW模块相关
add_subdirectory(src_object/Polygon)

add_library(utils SHARED
        src_object/utils/matrix_utils.cpp
        src_object/utils/dataprocess_utils.cpp
        src_object/utils/file_operate.cpp
)

target_link_libraries(utils
        ${OpenCV_LIBS}
)

find_package(Boost REQUIRED COMPONENTS filesystem system)


set(SOURCE_LZW_QUADRIC_FILES
        src_object/MapObject.cpp
        src_object/Converter.cc
        src_object/core/Plane.cpp
        #      src_object/core/ConstrainPlane.cpp
        #      src_object/core/Ellipsoid.cpp
)


# 二、GTSAM-QUADRIC 相关  
set(SOURCE_GTSAM_QUADRIC_FILES
        ./src_gtsam_quadrics/Utilities.cpp
        ./src_gtsam_quadrics/ConstrainedDualQuadric.cpp
        ./src_gtsam_quadrics/AlignedBox2.cpp
        ./src_gtsam_quadrics/AlignedBox3.cpp
        ./src_gtsam_quadrics/BoundingBoxFactor.cpp
        # ./src_gtsam_quadrics/QuadricAngleFactor.cpp
        ./src_gtsam_quadrics/QuadricCamera.cpp
        ./src_gtsam_quadrics/DualConic.cpp
        ./src_gtsam_quadrics/BboxCameraFactor.cpp
        ./src_gtsam_quadrics/InnerBboxCameraFactor.cpp
        ./src_gtsam_quadrics/HeightCameraFactor.cpp
)



add_library(CONVENIENCE_LIB_NAME SHARED
        ${SOURCE_LZW_QUADRIC_FILES}
        ${SOURCE_GTSAM_QUADRIC_FILES}
)

target_link_libraries(CONVENIENCE_LIB_NAME
        ${catkin_LIBRARIES} ${EIGEN3_LIBS} ${GTSAM_LIBRARIES} gpmp2 ${OpenCV_LIBS}
)



# 三、测试各个GTSAM模块
# add_executable(TestGtasmFactor
#         src_example/TestGtasmFactor.cpp
#         ${SOURCE_LZW_QUADRIC_FILES}
#         # src_object/
# )
# target_link_libraries(TestGtasmFactor
#         ${catkin_LIBRARIES} ${EIGEN3_LIBS}  ${OpenCV_LIBS}
#         utils
#         Polygon
#         # ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
#         Boost::filesystem
#         Boost::system
#         ${GTSAM_LIBRARIES} gpmp2
#         ${CONVENIENCE_LIB_NAME}
# )
#add_executable(TestBboxEllipsoidFactor
#        src_example/TestBboxEllipsoidFactor.cpp
#        ${SOURCE_LZW_QUADRIC_FILES}
#        ${SOURCE_GTSAM_QUADRIC_FILES}
#)
#target_link_libraries(TestBboxEllipsoidFactor
#        ${catkin_LIBRARIES} ${EIGEN3_LIBS}  ${OpenCV_LIBS}
#        utils
#        Polygon
#        # ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
#        Boost::filesystem
#        Boost::system
#        ${GTSAM_LIBRARIES} gpmp2
#        ${CONVENIENCE_LIB_NAME}
#)
#add_executable(TestBboxCameraFactor
#        src_example/TestBboxCameraFactor.cpp
#
#)
#target_link_libraries(TestBboxCameraFactor
#        ${catkin_LIBRARIES} ${EIGEN3_LIBS}  ${OpenCV_LIBS}
#        utils
#        Polygon
#        # ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
#        Boost::filesystem
#        Boost::system
#        ${GTSAM_LIBRARIES} gpmp2
#        ${CONVENIENCE_LIB_NAME}
#        ${OpenCV_LIBS}
#)


# 四、GTSAM与WAM结合

# message(STATUS "gpmp_wam_Reproduce_Matlab")
# add_executable(gpmp_wam_Reproduce_Matlab src_wam/gpmp_wam_Reproduce_Matlab.cpp )
# target_link_libraries(gpmp_wam_Reproduce_Matlab   ${catkin_LIBRARIES} ${EIGEN3_LIBS} ${GTSAM_LIBRARIES} gpmp2 ${OpenCV_LIBS})  #${Boost_LIBRARIES}


# message(STATUS "gpmp_wam_getfromik")
# add_executable(wam_getfromik src_wam/wam_getfromik_debug.cpp )
# target_link_libraries(wam_getfromik   ${catkin_LIBRARIES} ${EIGEN3_LIBS} ${GTSAM_LIBRARIES} gpmp2 ${OpenCV_LIBS})  #${Boost_LIBRARIES}

# message(STATUS "my_wam_gpmp")
# add_executable(wam_gpmp src_wam/wam_gpmp.cpp   )
# target_link_libraries(wam_gpmp   ${CONVENIENCE_LIB_NAME})  #${Boost_LIBRARIES}

 add_executable(wam_bboxcamera src_wam/wam_bboxcamera.cpp
#         ${SOURCE_LZW_QUADRIC_FILES}
#         ${SOURCE_GTSAM_QUADRIC_FILES}
 )
 target_link_libraries(wam_bboxcamera
         ${catkin_LIBRARIES} ${EIGEN3_LIBS} ${GTSAM_LIBRARIES} gpmp2 ${OpenCV_LIBS}  CONVENIENCE_LIB_NAME )
