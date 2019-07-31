# ---------------------------------------------------------------------------
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.
# The ASF licenses this file to You under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with
# the License.  You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ---------------------------------------------------------------------------

CMAKE_MINIMUM_REQUIRED(VERSION 2.8 FATAL_ERROR)

PROJECT(uimacpp_ros)

FIND_PACKAGE(catkin REQUIRED)

ADD_DEFINITIONS(-DUIMA_VERSION=2.4.1)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ADD_DEFINITIONS(-DCOMPILEDWITHC11)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -DNDEBUG -DTRACEOFF -DLINUX=2 -D_REENTRANT -D_GNU_SOURCE -D_LARGEFILE64_SOURCE")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG -DTRACEOFF -DLINUX=2 -D_REENTRANT -D_GNU_SOURCE -D_LARGEFILE64_SOURCE")

LIST(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")


FIND_PACKAGE(XercesC)
IF(XercesC_FOUND)
  INCLUDE_DIRECTORIES(${XercesC_INCLUDE_DIRS})
ELSE()
  MESSAGE(FATAL_ERROR "XercesC library not found")
ENDIF(XercesC_FOUND)

FIND_PACKAGE(APR)
IF(APR_FOUND)
  INCLUDE_DIRECTORIES(${APR_INCLUDE_DIR}, ${APRUTIL_INCLUDE_DIR})
ELSE()
  MESSAGE(FATAL_ERROR "APR library not found")
ENDIF(APR_FOUND)

FIND_PACKAGE(JNI)
IF(JNI_FOUND)
  INCLUDE_DIRECTORIES(${JNI_INCLUDE_DIRS})
ELSE()
  MESSAGE(FATAL_ERROR "JNI library not found")
ENDIF(JNI_FOUND)

FIND_PACKAGE(ICU)
IF(ICU_FOUND)
  INCLUDE_DIRECTORIES(${ICU_INCLUDE_DIRS})
ELSE()
  MESSAGE(FATAL_ERROR "ICU library not found")
ENDIF(ICU_FOUND)


# Set exported headers and libs
catkin_package(
  INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/cas/include ${CMAKE_CURRENT_SOURCE_DIR}/src/framework/include ${CMAKE_CURRENT_SOURCE_DIR}/src/jni/include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  DEPENDS XercesC APR JNI ICU)

ADD_SUBDIRECTORY(src)
