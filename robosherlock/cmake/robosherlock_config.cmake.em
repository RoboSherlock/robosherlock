set(robosherlock_NAMESPACE rs)
@[if DEVELSPACE]@

set(robosherlock_TYPESYSTEM_CPP_PATH "@(PROJECT_SOURCE_DIR)/src/core/include/robosherlock/types")
set(robosherlock_TYPESYSTEM_XML_PATH "@(PROJECT_SOURCE_DIR)/descriptors/typesystem")
set(robosherlock_ANNOTATOR_PATH      "@(PROJECT_SOURCE_DIR)/descriptors/annotators")
set(robosherlock_ENGINE_PATH         "@(PROJECT_SOURCE_DIR)/descriptors/analysis_engines")
set(RS_SCRIPT_PATH "@(PROJECT_SOURCE_DIR)/scripts")
set(RS_PROJECT_CONFIG "@(PROJECT_SOURCE_DIR)/cmake/project_config.cmake.in")
@[else]@
set(robosherlock_TYPESYSTEM_CPP_PATH "${robosherlock_PREFIX}/@(CATKIN_PACKAGE_INCLUDE_DESTINATION)/types")
set(robosherlock_TYPESYSTEM_XML_PATH "${robosherlock_PREFIX}/@(CATKIN_PACKAGE_SHARE_DESTINATION)/descriptors/typesystem")
set(robosherlock_ANNOTATOR_PATH      "${robosherlock_PREFIX}/@(CATKIN_PACKAGE_SHARE_DESTINATION)/descriptors/annotators")
set(robosherlock_ENGINE_PATH         "${robosherlock_PREFIX}/@(CATKIN_PACKAGE_SHARE_DESTINATION)/descriptors/analysis_engines")
set(RS_SCRIPT_PATH         "${robosherlock_PREFIX}/@(CATKIN_PACKAGE_SHARE_DESTINATION)/scripts")
set(RS_PROJECT_CONFIG      "${robosherlock_PREFIX}/@(CATKIN_PACKAGE_SHARE_DESTINATION)/cmake/project_config.cmake.in")
@[end if]@
