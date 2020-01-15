project(rs_core)

add_library(rs_core SHARED
  src/DrawingAnnotator.cpp
  src/scene_cas.cpp
  src/conversion/bson.cpp
  src/conversion/bson_conversion.cpp
  src/conversion/json.cpp
  src/conversion/json_conversion.cpp
  src/conversion/conversion.cpp
  src/conversion/cv_conversion.cpp
  src/conversion/pcl_conversion.cpp
  src/conversion/ros_conversion.cpp
  src/conversion/segmentation_conversion.cpp
  src/conversion/tf_conversion.cpp
)
target_link_libraries(rs_core
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${mongoClient_LIBRARIES}
)

#add_dependencies(rs_core robosherlock_generate_type_system)

install(
  TARGETS rs_core 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

