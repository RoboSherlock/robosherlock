# UIMA_LIBRARY_FOUND - system has UIMA
# UIMA_LIBRARY - UIMA Library

find_library(UIMA_LIB
  NAMES libuima.so
  DOC "Found UIMA library"
  PATHS /usr/local/lib
)

if(UIMA_LIB)
  set(UIMA_LIBRARY ${UIMA_LIB})
  set(UIMA_LIBRARIES ${UIMA_LIB})
  message(STATUS "Found UIMA Library: ${UIMA_LIB}")
  set(UIMA_LIBRARY_FOUND true)
else()
  set(UIMA_LIBRARY_FOUND false)
  if(UIMA_LIBRARY_FIND_REQUIRED)
    message(FATAL_ERROR "UIMA library not found!")
  else()
    message(STATUS "UIMA library not found!")
  endif()
endif()

