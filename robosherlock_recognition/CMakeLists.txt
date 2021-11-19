cmake_minimum_required(VERSION 2.8.3)
project(robosherlock_recognition)
find_package(catkin REQUIRED robosherlock)
################################################################################
## Constants for project                                                      ##
################################################################################
set(NAMESPACE robosherlock_recognition)
set(TYPESYSTEM_CPP_PATH ${PROJECT_SOURCE_DIR}/include/robosherlock_recognition/types)
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

add_library(rs_recognition SHARED src/LinemodInterface.cpp)
target_link_libraries(rs_recognition ${OpenCV_LIBRARIES})

rs_add_library(rs_LinemodAnnotator src/LinemodAnnotator.cpp)
target_link_libraries(rs_LinemodAnnotator  rs_recognition)

if(LIBPRACMLN)
  rs_add_library(rs_MLNInferencer src/MLNInferencer.cpp)
  target_link_libraries(rs_MLNInferencer  ${LIBPRACMLN})
endif(LIBPRACMLN)

if(Caffe_FOUND)
  add_library(rs_caffeProxy src/CaffeProxy.cpp)
  target_link_libraries(rs_caffeProxy ${catkin_LIBRARIES} ${Caffe_LIBRARIES})
  install(TARGETS rs_caffeProxy LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
  set(RS_CAFFE_LIB rs_caffeProxy)
endif(Caffe_FOUND)

install(
  TARGETS rs_recognition
          rs_LinemodAnnotator           
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
) 
