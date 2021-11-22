
#############################################################################
## Check for c++14 support                                                 ##
#############################################################################
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX14)
if(COMPILER_SUPPORTS_CXX14)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
else()
  message(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

#############################################################################
## Set cxx flags                                                           ##
#############################################################################
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBT_USE_DOUBLE_PRECISION -Wno-deprecated")
# Unused warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wuninitialized -Winit-self -Wunused-function -Wunused-label -Wunused-variable -Wunused-but-set-parameter")#-Wunused-but-set-variable
# Additional warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wreorder -Warray-bounds -Wtype-limits -Wreturn-type -Wsequence-point -Wparentheses -Wmissing-braces -Wchar-subscripts -Wswitch -Wwrite-strings -Wenum-compare -Wempty-body")# -Wlogical-op")

#############################################################################
## Check for openMP support                                                ##
#############################################################################
find_package(OpenMP)
if(OPENMP_FOUND)
  message(STATUS "OpenMP found.")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


##############################################################################
### Check for Caffe Support                                                 ##
##############################################################################
find_package(Caffe QUIET)
if(Caffe_FOUND)
  message(STATUS "Caffe FOUND! Building related components")

  include_directories(${Caffe_INCLUDE_DIRS})
  add_definitions( -DCAFFE_FOUND ${Caffe_DEFINITIONS})

  set(RS_CAFFE_LIB rs_caffeProxy)
  set(RS_WITH_CAFFE TRUE)
else()
  set(RS_WITH_CAFFE FALSE)
endif(Caffe_FOUND)


#############################################################################
## Check for Protobuf Support                                              ##
#############################################################################
find_package(Protobuf QUIET)
if(PROTOBUF_FOUND AND PROTOBUF_PROTOC_EXECUTABLE)
  message (STATUS "Found Protobuf and Protobuf executable.")	
  set(RS_WEB_LIB rs_web)
  set(RS_WITH_GG TRUE)
else()
  set(RS_WITH_GG FALSE)
endif()


#############################################################################
## Adding include dirs defined in CPATH (needed for xstow)                 ##
#############################################################################
if($ENV{CPATH})
  string(REPLACE ":" ";" paths $ENV{CPATH})
  foreach(path ${paths})
    include_directories(${path})
  endforeach()
endif()

#############################################################################
## Add library macro                                                       ##
#############################################################################
macro(rs_add_library libname)
  add_library(${libname} SHARED ${ARGN})
  target_link_libraries(${libname} ${LIBAPR_LIBRARY} ${UIMA_LIBRARY} ${ICUUC_LIBRARY} ${catkin_LIBRARIES})
  set_target_properties(${libname} PROPERTIES PREFIX "")
endmacro(rs_add_library)

#############################################################################
## Add executable macro                                                    ##
#############################################################################
macro(rs_add_executable execname)
  add_executable(${execname} ${ARGN})
  target_link_libraries(${execname} ${LIBAPR_LIBRARY} ${UIMA_LIBRARY} ${ICUUC_LIBRARY} ${catkin_LIBRARIES})
endmacro(rs_add_executable)

#############################################################################
## Generate classes from the typesystem xml files                          ##
#############################################################################
macro(generate_type_system)
  set(script ${RS_SCRIPT_PATH}/generate_typesystem.py)
  set(projects ${PROJECT_NAME}:${NAMESPACE}:${TYPESYSTEM_XML_PATH}:${TYPESYSTEM_CPP_PATH})

  foreach(arg ${ARGN}) 
    set(projects ${projects} ${arg}:${${arg}_NAMESPACE}:${${arg}_TYPESYSTEM_XML_PATH}:${${arg}_TYPESYSTEM_CPP_PATH})
  endforeach()
  
  execute_process(COMMAND python3 ${script} ${projects})
endmacro(generate_type_system)

#############################################################################
## Find all include directories for export                                 ##
#############################################################################
macro(find_include_dirs result)
  set(search_pattern -name "include" -type d)

  foreach(arg ${ARGN})
    set(exclude ${exclude} ! -path "${PROJECT_SOURCE_DIR}/${arg}/*")
  endforeach()

  message(STATUS "find path: ${PROJECT_SOURCE_DIR}/${arg}")

  execute_process(COMMAND find "-L" ${PROJECT_SOURCE_DIR} ${exclude} ${search_pattern} OUTPUT_VARIABLE found_include_dirs)
  message(STATUS "executed: find ${PROJECT_SOURCE_DIR} ${exclude} ${search_pattern}")

  message(STATUS "found include path: ${found_include_dirs}")

  if(NOT "${found_include_dirs}" STREQUAL "")
    string(REPLACE "\n" "/\;" ${result} ${found_include_dirs})
    #message(STATUS "additional include dirs: ${${result}}")
  endif()
endmacro(find_include_dirs)

#############################################################################
## Hack for QT Creator to include all relevant files                       ##
#############################################################################
macro(find_additional_files)
  set(search_pattern -type f)

  foreach(arg ${ARGN}) 
    set(exclude ${exclude} ! -path "${PROJECT_SOURCE_DIR}/${arg}/*")
  endforeach()
  execute_process(COMMAND find "-L" ${PROJECT_SOURCE_DIR} ${exclude} ${search_pattern} OUTPUT_VARIABLE found_files)
  if(NOT "${found_files}" STREQUAL "")
    string(REPLACE "\n" ";" found_files_list ${found_files})
  add_custom_target(${PROJECT_NAME}_additional_files SOURCES ${found_files_list})
  endif()
endmacro(find_additional_files)

#############################################################################
## Message on option status                                                ##
#############################################################################
macro(check_option var)
  if(${var})
    message(STATUS "${var} activated")
  else(${var})
    message(STATUS "${var} deactivated")
  endif(${var})
endmacro(check_option)

