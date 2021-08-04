cmake_minimum_required(VERSION 2.8.3)
project(robosherlock_tracking)
find_package(catkin REQUIRED robosherlock)
################################################################################
## Constants for project                                                      ##
################################################################################
set(NAMESPACE robosherlock_tracking)
set(TYPESYSTEM_CPP_PATH ${PROJECT_SOURCE_DIR}/include/robosherlock_tracking/types)
set(TYPESYSTEM_XML_PATH ${PROJECT_SOURCE_DIR}/descriptors/typesystem)
set(ANNOTATOR_PATH      ${PROJECT_SOURCE_DIR}/descriptors/annotators)
set(ENGINE_PATH         ${PROJECT_SOURCE_DIR}/descriptors/analysis_engines)
set(CMAKE_MODULE_PATH   ${PROJECT_SOURCE_DIR}/cmake)
################################################################################
## Configure project configuration files                                      ##
################################################################################
configure_file(${RS_PROJECT_CONFIG} ${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}_config.cmake)
################################################################################
## Find all include directories                                               ##
################################################################################
find_include_dirs(RS_INCLUDE_DIRS_LIST)
catkin_package(
   CFG_EXTRAS ${PROJECT_NAME}_config.cmake
   )
################################################################################
## Package dependencies                                                       ##
################################################################################
#Add required catkin packages here
#find_package(catkin REQUIRED COMPONENTS xxx)
#Add required system packages here
#find_package(xxx)
################################################################################
## Build                                                                      ##
################################################################################
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        ${RS_INCLUDE_DIRS_LIST}
        ${catkin_INCLUDE_DIRS}
        )
################################################################################
## Update analysis engines, typesystem and include all relevant files         ##
################################################################################
## generate classes from the typesystem xml files
generate_type_system(robosherlock)
#find all relevant files
find_additional_files()
################################################################################
## Sub Projects                                                               ##
################################################################################
#If you want to divide your projects into subprojects include the subdirectories
#each containing a CMakeLists.txt here
#add_subdirectory(src/xxx)


rs_add_library(rs_ObjectIdentityResolution src/ObjectIdentityResolution.cpp)
target_link_libraries(rs_ObjectIdentityResolution rs_core)

rs_add_library(rs_ClusterTracker src/ClusterTracker.cpp)
target_link_libraries(rs_ClusterTracker rs_core )

if(OpenCV_VERSION VERSION_GREATER "3.3")
  rs_add_library(rs_KCFTrackingAnnotator src/KCFTrackingAnnotator.cpp)
  target_link_libraries(rs_KCFTrackingAnnotator ${catkin_LIBRARIES})
endif()

rs_add_library(rs_PCLParticleTrackingAnnotator src/PCLParticleTrackingAnnotator.cpp)
target_link_libraries(rs_PCLParticleTrackingAnnotator ${catkin_LIBRARIES})


install(
  TARGETS rs_ObjectIdentityResolution
          rs_ClusterTracker
          rs_PCLParticleTrackingAnnotator
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
