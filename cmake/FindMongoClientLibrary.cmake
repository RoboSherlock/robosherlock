# MONGO_CLIENT_LIBRARY_FOUND - system has Mongo Client
# MONGO_CLIENT_LIBRARY - Mongo Client Library

message(STATUS "Looking for Mongo Client Library...")

find_library(MONGO_CLIENT_LIBRARY
  NAMES libmongoclient.so
  DOC "Found MONGO library"
  PATHS /usr/local/lib
)

if(MONGO_CLIENT_LIBRARY)
  set(MONGO_CLIENT_LIBRARY_FOUND 1)
        
  if(NOT MONGO_CLIENT_LIBRARY_FIND_QUIETLY)
    message(STATUS "Found Mongo CLient Library: ${MONGO_CLIENT_LIBRARY}")
  endif(NOT MONGO_CLIENT_LIBRARY_FIND_QUIETLY)

else(MONGO_CLIENT_LIBRARY)

  set(MONGO_CLIENT_LIBRARY_FOUND 0 CACHE BOOL "Mongo Client Library not found")
  if(MONGO_CLIENT_LIBRARY_FIND_REQUIRED)
    message(FATAL_ERROR "Mongo Client Library not found, error")
  else()
    MESSAGE(STATUS "Mongo Client Library not found, disabled")
  endif()
endif(MONGO_CLIENT_LIBRARY)

MARK_AS_ADVANCED(MONGO_CLIENT_LIBRARY)


