cmake_minimum_required(VERSION 2.8.3)
project(robosherlock_annotations)
find_package(catkin REQUIRED robosherlock rapidjson_ros)
################################################################################
## Constants for project                                                      ##
################################################################################
set(NAMESPACE rs)
set(TYPESYSTEM_CPP_PATH ${PROJECT_SOURCE_DIR}/include/robosherlock_annotations/types)
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

add_definitions(-DRAPIDJSON_HAS_STDSTRING)

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
generate_type_system(robosherlock )
#find all relevant files
find_additional_files()
################################################################################
## add library                                                                ##
################################################################################

rs_add_library(rs_Cluster3DGeometryAnnotator src/Cluster3DGeometryAnnotator.cpp)
target_link_libraries(rs_Cluster3DGeometryAnnotator ${catkin_LIBRARIES})

rs_add_library(rs_ClusterColorHistogramCalculator src/ClusterColorHistogramCalculator.cpp)
target_link_libraries(rs_ClusterColorHistogramCalculator ${catkin_LIBRARIES} )

rs_add_library(rs_ClusterTFLocationAnnotator src/ClusterTFLocationAnnotator.cpp)
target_link_libraries(rs_ClusterTFLocationAnnotator ${catkin_LIBRARIES})

rs_add_library(rs_FeatureAnnotator src/FeatureAnnotator.cpp)
target_link_libraries(rs_FeatureAnnotator ${catkin_LIBRARIES})

rs_add_library(rs_NormalEstimator src/NormalEstimator.cpp)
target_link_libraries(rs_NormalEstimator ${catkin_LIBRARIES})

rs_add_library(rs_PrimitiveShapeAnnotator src/PrimitiveShapeAnnotator.cpp)
target_link_libraries(rs_PrimitiveShapeAnnotator ${catkin_LIBRARIES} )

rs_add_library(rs_SacModelAnnotator src/SacModelAnnotator.cpp)
target_link_libraries(rs_SacModelAnnotator ${catkin_LIBRARIES})

rs_add_library(rs_SuperVoxelAnnotator src/SuperVoxelAnnotator.cpp)
target_link_libraries(rs_SuperVoxelAnnotator ${catkin_LIBRARIES})

rs_add_library(rs_pclDescriptorExtractor src/PCLDescriptorExtractor.cpp)
target_link_libraries(rs_pclDescriptorExtractor ${catkin_LIBRARIES})


################################################################################
## Install                                                                    ##
################################################################################
install(
  TARGETS rs_Cluster3DGeometryAnnotator
          rs_ClusterColorHistogramCalculator
          rs_ClusterTFLocationAnnotator
          rs_FeatureAnnotator
          rs_NormalEstimator
          rs_PrimitiveShapeAnnotator
          rs_SacModelAnnotator
          rs_SuperVoxelAnnotator
          rs_pclDescriptorExtractor
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

