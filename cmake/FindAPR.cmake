# APR_LIBRARY_FOUND - system has Mongo Client
# APR_LIBRARY - Mongo Client Library
# APR_INCLUDE_DIR - Mongo Client Library

find_package(PkgConfig)
pkg_check_modules(PC_LIBAPR QUIET apr-1)
set(LIBAPR_DEFINITIONS ${PC_LIBAPR_CFLAGS_OTHER})

find_path(LIBAPR_INCLUDE_DIR
  apr.h
  HINTS ${PC_LIBAPR_INCLUDEDIR} ${PC_LIBAPR_INCLUDE_DIRS}
  DOC "Found LIBAPR include directory"
)

find_library(LIBAPR_LIBRARY
  NAMES apr-1.0 apr-1 libapr-1.0 libapr-1
  HINTS ${PC_LIBAPR_LIBDIR} ${PC_LIBAPR_LIBRARY_DIRS}
  DOC "Found LIBAPR library"
)

set(LIBAPR_LIBRARIES ${LIBAPR_LIBRARY} )
set(LIBAPR_INCLUDE_DIRS ${LIBAPR_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LIBXML2_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(LibAPR1  DEFAULT_MSG
  LIBAPR_LIBRARY LIBAPR_INCLUDE_DIR)

mark_as_advanced(LIBAPR_INCLUDE_DIR LIBAPR_LIBRARY )
