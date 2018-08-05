project(rs_utils)

add_library(rs_utils SHARED
  src/DepthImageProcessing.cpp
  src/BlurDetector.cpp
  src/Graph.cpp
  src/WeightedGraph.cpp
  src/DirectedGraph.cpp
  src/GraphPrimitives.cpp
  src/YamlToXMLConverter.cpp
  src/RSXMLParser.cpp
)
target_link_libraries(rs_utils rs_core ${YAML_CPP_LIBRARIES} ${catkin_LIBRARIES} ${XercesC_LIBRARIES})

rs_add_library(rs_BlurDetectorAnnotator src/BlurDetectorAnnotator.cpp)
target_link_libraries(rs_BlurDetectorAnnotator rs_core rs_utils)

rs_add_library(rs_PointCloudDownsampler src/PointCloudDownsampler.cpp)
target_link_libraries(rs_PointCloudDownsampler rs_core)

rs_add_library(rs_ClusterMerger src/ClusterMerger.cpp)
target_link_libraries(rs_ClusterMerger rs_core)

rs_add_library(rs_ImagePreprocessor src/ImagePreprocessor.cpp)
target_link_libraries(rs_ImagePreprocessor rs_core rs_utils rs_segmentation)

rs_add_library(rs_Trigger src/Trigger.cpp)
target_link_libraries(rs_Trigger rs_core rs_io)

rs_add_executable(test_reconfig src/test_reconfig.cpp)
target_link_libraries(test_reconfig rs_flowcontrol)
