project(rs_core)

add_library(rs_core SHARED
  src/DrawingAnnotator.cpp
  src/scene_cas.cpp
  src/conversion/bson.cpp
  src/conversion/bson_conversion.cpp
#  src/conversion/json.cpp
#  src/conversion/json_conversion.cpp

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
)
add_dependencies(rs_core robosherlock_generate_type_system)

add_library(rs_json_conversion SHARED src/conversion/json.cpp src/conversion/json_conversion.cpp)
target_link_libraries(rs_json_conversion rs_core ${catkin_LIBRARIES})
add_dependencies(rs_json_conversion robosherlock_generate_type_system)
