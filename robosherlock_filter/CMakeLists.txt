cmake_minimum_required(VERSION 2.8.3)
project(robosherlock_filter)
find_package(catkin REQUIRED robosherlock rapidjson_ros)
################################################################################
## Constants for project                                                      ##
################################################################################
set(NAMESPACE robosherlock_filter)
set(TYPESYSTEM_CPP_PATH ${PROJECT_SOURCE_DIR}/include/robosherlock_filter/types)
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


#find_package(RapidJSON REQUIRED)
#if(${RapidJSON_FOUND})
add_definitions(-DRAPIDJSON_HAS_STDSTRING)
#endif()

####################################################################
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

rs_add_library(rs_PointCloudFilter src/PointCloudFilter.cpp)
target_link_libraries(rs_PointCloudFilter ${catkin_LIBRARIES})

rs_add_library(rs_RegionFilter src/RegionFilter.cpp)
target_link_libraries(rs_RegionFilter ${YAML_CPP_LIBRARIES} ${catkin_LIBRARIES})

rs_add_library(rs_ClusterFilter src/ClusterFilter.cpp)
target_link_libraries(rs_ClusterFilter ${catkin_LIBRARIES})


install(
  TARGETS rs_PointCloudFilter
          rs_RegionFilter
          rs_ClusterFilter 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
