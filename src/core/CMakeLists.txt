project(rs_core)

add_library(rs_core SHARED
  src/DrawingAnnotator.cpp
  src/scene_cas.cpp
  src/conversion/bson.cpp
  src/conversion/bson_conversion.cpp
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
  ${UIMA_LIBRARY}
  ${MONGO_CLIENT_LIBRARY}
)
add_dependencies(rs_core generate_type_system)
