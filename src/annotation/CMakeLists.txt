project(rs_annotation)

rs_add_library(rs_Cluster3DGeometryAnnotator src/Cluster3DGeometryAnnotator.cpp)
target_link_libraries(rs_Cluster3DGeometryAnnotator rs_core)

rs_add_library(rs_ClusterColorHistogramCalculator src/ClusterColorHistogramCalculator.cpp)
target_link_libraries(rs_ClusterColorHistogramCalculator rs_core )

rs_add_library(rs_ClusterTFLocationAnnotator src/ClusterTFLocationAnnotator.cpp)
target_link_libraries(rs_ClusterTFLocationAnnotator rs_core)

rs_add_library(rs_FeatureAnnotator src/FeatureAnnotator.cpp)
target_link_libraries(rs_FeatureAnnotator rs_core)

rs_add_library(rs_NormalEstimator src/NormalEstimator.cpp)
target_link_libraries(rs_NormalEstimator rs_core)

rs_add_library(rs_PCLVFHFeatureExtractor src/PCLVFHFeatureExtractor.cpp)
target_link_libraries(rs_PCLVFHFeatureExtractor rs_core )

rs_add_library(rs_PrimitiveShapeAnnotator src/PrimitiveShapeAnnotator.cpp)
target_link_libraries(rs_PrimitiveShapeAnnotator rs_core )

rs_add_library(rs_SacModelAnnotator src/SacModelAnnotator.cpp)
target_link_libraries(rs_SacModelAnnotator rs_core)

rs_add_library(rs_SuperVoxelAnnotator src/SuperVoxelAnnotator.cpp)
target_link_libraries(rs_SuperVoxelAnnotator rs_core)

rs_add_library(rs_pclDescriptorExtractor src/PCLDescriptorExtractor.cpp)
target_link_libraries(rs_pclDescriptorExtractor rs_core)


#build the web annotations (Goggles only atm)

find_package(Protobuf QUIET)

if(PROTOBUF_FOUND)
  message(STATUS "Found Protobuf.")
endif()
if(PROTOBUF_PROTOC_EXECUTABLE)
  message(STATUS "Found Protobuf executeable.")
endif()


if(PROTOBUF_FOUND AND PROTOBUF_PROTOC_EXECUTABLE)

  message("Found Protobuf. Building Goggles")
  find_package(PythonLibs)

  include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${Boost_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
  )

  PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS
    include/rs/annotation/web/request.proto
    include/rs/annotation/web/response.proto
    )

  add_library(rs_web SHARED
      src/web/goggles.cpp
      ${PROTO_SRCS}
    )
  target_link_libraries(rs_web
      ${OpenCV_LIBRARIES}
      ${Boost_LIBRARIES}
      ${PROTOBUF_LIBRARY}
      ${catkin_LIBRARIES}
      ${PYTHON_LIBRARIES}
  )

  rs_add_library(rs_gogglesAnnotator src/web/GogglesAnnotator.cpp)
  target_link_libraries(rs_gogglesAnnotator rs_core rs_web)
endif()
