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

# Code Coverage
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MY_CMAKE_CXX_FLAGS}")

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
 visualization_msgs cv_bridge image_transport
 image_geometry message_filters resource_retriever
 robosherlock_msgs std_srvs libmongocxx_ros
 uimacpp_ros rapidjson_ros)


find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  #Fallback to deprecated cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${Eigen_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${Eigen_LIBRAIES})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread date_time python)

find_package(APR REQUIRED)
find_package(ICUUC REQUIRED)

include(FindPkgConfig)
pkg_check_modules(SWIPL REQUIRED swipl)
include_directories(${SWIPL_INCLUDE_DIRS})
link_directories(${SWIPL_LIBRARY_DIRS})


find_library(LIBPRACMLN pracmln QUIET)
if(LIBPRACMLN)
  message(STATUS "libpracmln FOUND = " ${LIBPRACMLN})
else()
  message(STATUS "libpracmln NOT FOUND. Related annotators will not be build!")
endif()

find_package(Caffe QUIET)
if(Caffe_FOUND)
  message(STATUS "Caffe FOUND")
  set(OPTIONAL_Caffe Caffe)
  add_definitions( -DCAFFE_FOUND ${Caffe_DEFINITIONS})
  set(RS_CAFFE_LIB rs_caffeProxy)
  include_directories(${Caffe_INCLUDE_DIRS})
endif(Caffe_FOUND)

find_package(Protobuf QUIET)
if(PROTOBUF_FOUND)
  message(STATUS "Found Protobuf.")
endif()
if(PROTOBUF_PROTOC_EXECUTABLE)
  message(STATUS "Found Protobuf executeable.")
endif()
if(PROTOBUF_FOUND AND PROTOBUF_PROTOC_EXECUTABLE)
  set(RS_WEB_LIB rs_web)
endif()

#find_package(RapidJSON REQUIRED)
#if(${RapidJSON_FOUND})
    add_definitions(-DRAPIDJSON_HAS_STDSTRING)
#endif()

find_package(json_prolog QUIET)
option(WITH_JSON_PROLOG "build using json prolog" False)

if(json_prolog_FOUND OR ${WITH_JSON_PROLOG})
    message(STATUS "json_prolog FOUND")
    add_definitions(-DWITH_JSON_PROLOG)
    include_directories(${json_prolog_INCLUDE_DIRS})
else()
    message(STATUS "json_prolog NOT FOUND. No Queryanswering will be available.")
endif()



find_package(yaml-cpp REQUIRED)
if(yaml-cpp_FOUND)
  message(STATUS "yaml-cpp FOUND")
  include_directories(${YAML_CPP_INCLUDE_DIR})
else()
  message(STATUS "libyaml_cpp not found.")
endif()

################################################################################
## CMake variables                                                            ##
################################################################################

add_definitions(${PCL_DEFINITIONS})

################################################################################
## catkin specific configuration                                              ##
################################################################################

# libraries to export
set(RS_LIBS
  rs_core
  rs_flowcontrol
  rs_io
  rs_segmentation
  rs_symmetrysegmentation
  rs_recognition
  rs_queryanswering
  rs_utils
  ${RS_CAFFE_LIB}
  ${RS_WEB_LIB}
)

catkin_package(
  INCLUDE_DIRS ${RS_INCLUDE_DIRS_LIST} ${SWIPL_INCLUDE_DIRS} ${LIBAPR_INCLUDE_DIRS} ${Caffe_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
  LIBRARIES ${RS_LIBS}
  CATKIN_DEPENDS pcl_ros cv_bridge libmongocxx_ros robosherlock_msgs uimacpp_ros rapidjson_ros
  DEPENDS Boost OpenCV APR PCL ICUUC
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
  ${EIGEN3_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

## generate classes from the typesystem xml files
generate_type_system()

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
add_subdirectory(src/segmentation/symmetrysegmentation)
add_subdirectory(src/utils)
add_subdirectory(src/tracking)
add_subdirectory(src/flowcontrol)
add_subdirectory(src/queryanswering)
################################################################################
## Install                                                                    ##
################################################################################

################################################################################
## Testing                                                                    ##
################################################################################

catkin_add_gtest(${PROJECT_NAME}-test
    test/main.cpp
    test/robosherlock/test_swipl_interface.cpp
    test/robosherlock/test_queryinterface.cpp
    test/robosherlock/test_parallelism.cpp
    test/annotators_utest/symmetry_segmentation_test.cpp
    test/annotators_utest/test_storage.cpp
    test/annotators_utest/ImagePreprocessor_utest.cpp
    test/annotators_utest/PlaneAnnotator_utest.cpp
    test/annotators_utest/PointCloudClusterExtractor_utest.cpp
    test/annotators_utest/ImageSegmentationAnnotator_utest.cpp
    test/annotators_utest/Cluster3DGeometry_utest.cpp
    test/annotators_utest/ClusterColorHistogramCalculator_utest.cpp
    test/annotators_utest/PrimitiveShapesAnnotator_utest.cpp
    test/annotators_utest/FeatureAnnotator_utest.cpp
    test/annotators_utest/SacModelAnnotator_utest.cpp
    #test/annotators_utest/MLNInferencer_utest.cpp
    test/annotators_utest/PCLDescriptorExtractor_utest.cpp
    test/annotators_utest/reconf_utest.cpp)
target_link_libraries(${PROJECT_NAME}-test
    rs_queryanswering
    rs_flowcontrol
    rs_core
    rs_io
    rs_utils)
