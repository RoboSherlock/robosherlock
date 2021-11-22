project(rs_segmentation)

add_library(rs_segmentation SHARED src/ImageSegmentation.cpp)
target_link_libraries(rs_segmentation rs_core)

rs_add_library(rs_ImageSegmentationAnnotator src/ImageSegmentationAnnotator.cpp)
target_link_libraries(rs_ImageSegmentationAnnotator rs_core rs_segmentation)

rs_add_library(rs_PointCloudClusterExtractor src/PointCloudClusterExtractor.cpp)
target_link_libraries(rs_PointCloudClusterExtractor rs_core)

rs_add_library(rs_PointCloudColorSegmentation src/PointCloudColorSegmentation.cpp)
target_link_libraries(rs_PointCloudColorSegmentation rs_core)

rs_add_library(rs_transparentSegmentationAnnotator src/TransparentSegmentationAnnotator.cpp)
target_link_libraries(rs_transparentSegmentationAnnotator rs_core rs_segmentation)


install(
        TARGETS rs_segmentation
        rs_ImageSegmentationAnnotator
        rs_PointCloudClusterExtractor
        rs_PointCloudColorSegmentation
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)