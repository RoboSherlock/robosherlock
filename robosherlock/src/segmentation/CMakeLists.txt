project(rs_segmentation)

add_library(rs_segmentation SHARED src/ImageSegmentation.cpp)
target_link_libraries(rs_segmentation rs_core)

rs_add_library(rs_ImageSegmentationAnnotator src/ImageSegmentationAnnotator.cpp)
target_link_libraries(rs_ImageSegmentationAnnotator rs_core rs_segmentation)



install(
  TARGETS rs_segmentation
          rs_ImageSegmentationAnnotator
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
