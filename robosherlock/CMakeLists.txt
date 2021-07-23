cmake_minimum_required(VERSION 2.8.3)

project(robosherlock)

################################################################################
## Include RoboSherlock cmake file, containing macros                         ##
################################################################################
include(cmake/robosherlock.cmake)

################################################################################
## Constants for project                                                      ##
################################################################################
set(NAMESPACE rs)
set(TYPESYSTEM_CPP_PATH ${PROJECT_SOURCE_DIR}/src/core/include/robosherlock/types)
set(TYPESYSTEM_XML_PATH ${PROJECT_SOURCE_DIR}/descriptors/typesystem)
set(ANNOTATOR_PATH      ${PROJECT_SOURCE_DIR}/descriptors/annotators)
set(ENGINE_PATH         ${PROJECT_SOURCE_DIR}/descriptors/analysis_engines)
set(CMAKE_MODULE_PATH   ${PROJECT_SOURCE_DIR}/cmake)
set(RS_SCRIPT_PATH      ${PROJECT_SOURCE_DIR}/scripts)
set(RS_PROJECT_CONFIG   ${PROJECT_SOURCE_DIR}/cmake/project_config_cmake.in)

# Code Coverage
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MY_CMAKE_CXX_FLAGS}")


#the *.em file took the place of this. 
################################################################################
## Configure project configuration files                                      ##
################################################################################
#configure_file(${PROJECT_SOURCE_DIR}/cmake/project_config.cmake.in ${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}_config.cmake @ONLY)

################################################################################
## Find all include directories                                               ##
################################################################################
find_include_dirs(RS_INCLUDE_DIRS_LIST)

################################################################################
## Package dependencies                                                       ##
################################################################################

#mongo client
find_package(libmongocxx_ros QUIET)
if(NOT libmongocxx_ros_FOUND)
  find_package(MongoClientLibrary)
  if(NOT MONGO_CLIENT_LIBRARY_FOUND)
    message(FATAL_ERROR "Mongo not found. Search for libmongocxx_ros or MongoClientLibrary was not successful")
  else()
    message(STATUS "Found System mongo.")
    set(MONGO_SYSTEM ${MongoClientLibrary})
    set(mongoClient_LIBRARIES ${MONGO_CLIENT_LIBRARY})
  endif()
else()
  message(STATUS "libmongocxx_ros package found") 
  set(OPTIONAL_libmongocxx libmongocxx_ros)
  include_directories(${libmongocxx_ros_INCLUDE_DIRS})
endif() 

find_package(catkin REQUIRED
 COMPONENTS
 roscpp roslib pcl_ros pcl_conversions pcl_msgs
 tf tf_conversions rostime sensor_msgs std_msgs
 visualization_msgs cv_bridge image_transport
 image_geometry message_filters resource_retriever
 robosherlock_msgs std_srvs
 uimacpp_ros rapidjson_ros
 ${OPTIONAL_libmongocxx})


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

string(REPLACE "." ";" VERSION_LIST ${SWIPL_VERSION})
list(GET VERSION_LIST 0 SWIPL_VERSION_MAJOR)
list(GET VERSION_LIST 1 SWIPL_VERSION_MINOR)
list(GET VERSION_LIST 2 SWIPL_VERSION_PATCH)
add_compile_options(-DSWIPL_VERSION=${SWIPL_VERSION_MAJOR})


find_library(LIBPRACMLN pracmln QUIET)
if(LIBPRACMLN)
  message(STATUS "libpracmln FOUND = " ${LIBPRACMLN})
else()
  message(STATUS "libpracmln NOT FOUND. Related annotators will not be build!")
endif()

#find_package(RapidJSON REQUIRED)
#if(${RapidJSON_FOUND})
add_definitions(-DRAPIDJSON_HAS_STDSTRING)
#endif()

find_package(rosprolog QUIET)
option(WITH_ROS_PROLOG "build using json prolog" False)

if(rosprolog_FOUND OR ${WITH_ROS_PROLOG})
    message(STATUS "rosprolog FOUND")
    add_definitions(-DWITH_ROS_PROLOG)
    include_directories(${rosprolog_INCLUDE_DIRS})
else()
    message(STATUS "rosprolog NOT FOUND. using knowrob as the knowledge engine will not be possible")
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
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

################################################################################
## catkin specific configuration                                              ##
################################################################################

# libraries to export
set(RS_LIBS
  rs_core
  rs_io
  rs_queryanswering
  rs_segmentation
  rs_utils
  ${RS_WEB_LIB}
)

catkin_package(
  INCLUDE_DIRS ${RS_INCLUDE_DIRS_LIST} ${SWIPL_INCLUDE_DIRS} ${LIBAPR_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${libmongocxx_ros_INCLUDE_DIRS}
  LIBRARIES ${RS_LIBS}
  CATKIN_DEPENDS pcl_ros cv_bridge robosherlock_msgs uimacpp_ros rapidjson_ros
  DEPENDS Boost OpenCV APR PCL ICUUC ${MONGO_SYSTEM}
  CFG_EXTRAS robosherlock.cmake ${PROJECT_NAME}_config.cmake project_config.cmake.in
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


add_subdirectory(src/core)
add_subdirectory(src/flowcontrol)
add_subdirectory(src/io)
add_subdirectory(src/queryanswering)
add_subdirectory(src/segmentation)
add_subdirectory(src/utils)


################################################################################
## Testing                                                                    ##
################################################################################

catkin_add_gtest(${PROJECT_NAME}-test
        test/main.cpp
        test/robosherlock/test_casconsumercontext.cpp
        test/robosherlock/test_scene_cas.cpp
        test/robosherlock/test_swipl_interface.cpp
        test/robosherlock/test_queryinterface.cpp
        test/robosherlock/test_parallelism.cpp
        test/robosherlock/test_visualizables.cpp
        test/annotators_utest/test_storage.cpp
        test/annotators_utest/ImagePreprocessor_utest.cpp
        )
target_link_libraries(${PROJECT_NAME}-test
    rs_queryanswering
    rs_flowcontrol
    rs_core
    rs_io
    rs_utils)


################################################################################
## Install                                                                    ##
################################################################################
#append robosherlock to the path of the includes to make install path correct
string(REPLACE ";" "robosherlock/\;" RS_INSTALL_INCLUDE_DIRS ${RS_INCLUDE_DIRS_LIST}) 
install(DIRECTORY ${RS_INSTALL_INCLUDE_DIRS} DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#copy to descriptors and config folders to their appropriate locations
install(DIRECTORY descriptors DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY owl DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY prolog DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
