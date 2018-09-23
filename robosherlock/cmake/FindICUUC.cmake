# ICUUC_LIBRARY_FOUND - system has ICUUC
# ICUUC_LIBRARY - ICUUC Library

find_library(ICUUC_LIB
  NAMES libicuuc.so
  DOC "Found ICUUC library"
  PATHS /usr/local/lib
)

if(ICUUC_LIB)
  set(ICUUC_LIBRARY ${ICUUC_LIB})
  set(ICUUC_LIBRARIES ${ICUUC_LIB})
  message(STATUS "Found ICUUC Library: ${ICUUC_LIB}")
  set(ICUUC_LIBRARY_FOUND true)
else()
  set(ICUUC_LIBRARY_FOUND false)
  if(ICUUC_LIBRARY_FIND_REQUIRED)
    message(FATAL_ERROR "ICUUC library not found!")
  else()
    message(STATUS "ICUUC library not found!")
  endif()
endif()

