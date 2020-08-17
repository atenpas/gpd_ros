cmake_minimum_required(VERSION 2.8.3)
project(gpd_ros)

add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  eigen_conversions
  geometry_msgs
  message_generation
  pcl_conversions
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
)

# PCL
find_package(PCL REQUIRED)

## System dependencies are found with CMake's conventions
find_library(GPD_LIB NAMES gpd PATHS /usr/local/lib PATH_SUFFIXES lib NO_DEFAULT_PATH)
if (GPD_LIB)
  message(STATUS "Library GPD found in ${GPD_LIB}")
else()
  message(FATAL_ERROR "Library GPD not found")
endif()
include_directories(${GPD_LIB_INCLUDE_DIR})
message(STATUS "gpd_include_dir: ${GPD_LIB_INCLUDE_DIR}")

## Generate messages in the 'msg' folder
add_message_files(FILES
  CloudIndexed.msg
  CloudSamples.msg
  CloudSources.msg
  GraspConfig.msg
  GraspConfigList.msg
  SamplesMsg.msg
)

## Generate services in the 'srv' folder
add_service_files(FILES detect_grasps.srv)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

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
  LIBRARIES
    ${PROJECT_NAME}_grasp_messages
    ${PROJECT_NAME}_grasp_plotter
  CATKIN_DEPENDS
    cmake_modules
    eigen_conversions
    geometry_msgs
    message_generation
    message_runtime
    pcl_conversions
    roscpp
    sensor_msgs
    std_msgs
    visualization_msgs
  DEPENDS PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

## Declare a C++ library
add_library(${PROJECT_NAME}_grasp_messages src/${PROJECT_NAME}/grasp_messages.cpp)
add_library(${PROJECT_NAME}_grasp_plotter src/${PROJECT_NAME}/grasp_plotter.cpp)

## Add cmake target dependencies of the library (message generation needs to be built before libraries)
add_dependencies(${PROJECT_NAME}_grasp_messages ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(${PROJECT_NAME}_detect_grasps src/gpd_ros/grasp_detection_node.cpp)
add_executable(${PROJECT_NAME}_detect_grasps_server src/gpd_ros/grasp_detection_server.cpp)

## Rename C++ executable without prefix
set_target_properties(${PROJECT_NAME}_detect_grasps PROPERTIES OUTPUT_NAME detect_grasps PREFIX "")
set_target_properties(${PROJECT_NAME}_detect_grasps_server PROPERTIES OUTPUT_NAME detect_grasps_server PREFIX "")

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_detect_grasps
                      ${PROJECT_NAME}_grasp_messages
                      ${PROJECT_NAME}_grasp_plotter
                      ${GPD_LIB}
                      ${PCL_LIBRARIES}
                      ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_detect_grasps_server
                      ${PROJECT_NAME}_grasp_messages
                      ${PROJECT_NAME}_grasp_plotter
                      ${GPD_LIB}
                      ${PCL_LIBRARIES}
                      ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_grasp_plotter
                      ${GPD_LIB}
                      ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_grasp_messages
                      ${GPD_LIB}
                      ${catkin_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS
  ${PROJECT_NAME}_detect_grasps
  ${PROJECT_NAME}_detect_grasps_server
  ${PROJECT_NAME}_grasp_messages
  ${PROJECT_NAME}_grasp_plotter

  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN "*.git" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".git" EXCLUDE
)
