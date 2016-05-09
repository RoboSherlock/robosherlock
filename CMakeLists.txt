cmake_minimum_required(VERSION 2.8.3)

project(robosherlock)

################################################################################
## Include RoboSherlock cmake file                                            ##
################################################################################
configure_file(${PROJECT_SOURCE_DIR}/cmake/robosherlock.cmake.in ${PROJECT_SOURCE_DIR}/cmake/robosherlock.cmake @ONLY)
include(cmake/robosherlock.cmake)

################################################################################
## Constants for project                                                      ##
################################################################################
set(NAMESPACE rs)
set(TYPESYSTEM_CPP_PATH ${PROJECT_SOURCE_DIR}/src/core/include/rs/types)
set(TYPESYSTEM_XML_PATH ${PROJECT_SOURCE_DIR}/descriptors/typesystem)
set(ANNOTATOR_PATH      ${PROJECT_SOURCE_DIR}/descriptors/annotators)
set(ENGINE_PATH         ${PROJECT_SOURCE_DIR}/descriptors/analysis_engines)
set(CMAKE_MODULE_PATH   ${PROJECT_SOURCE_DIR}/cmake)

################################################################################
## Configure project configuration files                                      ##
################################################################################
configure_file(${RS_PROJECT_CONFIG} ${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}_config.cmake @ONLY)

################################################################################
## Find all include directories                                               ##
################################################################################

find_include_dirs(RS_INCLUDE_DIRS_LIST)

################################################################################
## CMake options                                                              ##
################################################################################

# Will not show up in any cmake gui. Only for internal use but it will overwrite
# RS_DEBUG_OUTPUT if set, that's why it's initialized empty
set(RS_OUT_LEVEL "" CACHE INTERNAL "Output-Level: Debug/\"3\", Info/\"2\", Error/\"1\", None/\"0\"")

option(RS_DEBUG_OUTPUT      "Enable debug output for non debug builds" OFF)

check_option(RS_DEBUG_OUTPUT)

################################################################################
## Package dependencies                                                       ##
################################################################################

find_package(catkin REQUIRED
 COMPONENTS
 roscpp roslib pcl_ros pcl_conversions pcl_msgs
 tf tf_conversions rostime sensor_msgs std_msgs
 cv_bridge image_transport image_geometry message_filters)
find_package(cmake_modules QUIET)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread date_time python)

find_package(APR REQUIRED)
find_package(MongoClientLibrary REQUIRED)
find_package(UIMA REQUIRED)
find_package(ICUUC REQUIRED)

find_library(LIBPRACMLN pracmln QUIET)
if(LIBPRACMLN)
  message(STATUS "libpracmln FOUND = " ${LIBPRACMLN})
else()
  message(STATUS "libpracmln NOT FOUND. Related annotators will not be build!")
endif()


################################################################################
## CMake variables                                                            ##
################################################################################

add_definitions(-DDB_SCRIPTS_DIR="${PROJECT_SOURCE_DIR}/scripts/mongodb/")
add_definitions(${PCL_DEFINITIONS})

################################################################################
## catkin specific configuration                                              ##
################################################################################

# libraries to export
set(RS_LIBS
  rs_core
  rs_io
  rs_segmentation
  rs_recognition
  rs_utils
  rs_analysisEngineManager
)

catkin_package(
  INCLUDE_DIRS ${RS_INCLUDE_DIRS_LIST} ${LIBAPR_INCLUDE_DIRS}
  LIBRARIES ${RS_LIBS}
  CATKIN_DEPENDS pcl_ros cv_bridge
  DEPENDS Boost OpenCV APR PCL Eigen UIMA ICUUC MongoClientLibrary
  CFG_EXTRAS robosherlock.cmake ${PROJECT_NAME}_config.cmake
)

################################################################################
## Set include directories                                                    ##
################################################################################

include_directories(
  ${RS_INCLUDE_DIRS_LIST}
  ${LIBAPR_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

################################################################################
## Update analysis engines, typesystem and include all relevant files         ##
################################################################################

## Update xml list of annotators inside analysis engines
update_analysis_engines(descriptors/analysis_engines)

## generate classes from the typesystem xml files
generate_type_system()

#find all relevant files
find_additional_files(3rdparty)

################################################################################
## Sub Projects                                                               ##
################################################################################

add_subdirectory(src/annotation)
add_subdirectory(src/core)
add_subdirectory(src/detection)
add_subdirectory(src/filter)
add_subdirectory(src/io)
add_subdirectory(src/recognition)
add_subdirectory(src/segmentation)
add_subdirectory(src/utils)
add_subdirectory(src/tracking)

################################################################################
## Install                                                                    ##
################################################################################

################################################################################
## Testing                                                                    ##
################################################################################
