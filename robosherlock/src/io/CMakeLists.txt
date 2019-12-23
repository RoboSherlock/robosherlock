project(rs_io)

add_library(rs_io SHARED
  src/CamInterface.cpp
  src/ROSCamInterface.cpp
  src/ROSKinectBridge.cpp
  src/ROSRealSenseBridge.cpp
  src/ROSTangoBridge.cpp
  src/ROSThermalCamBridge.cpp
  src/ROSCameraBridge.cpp
  src/UnrealVisionBridge.cpp
  src/UnrealROSIntegrationVisionBridge.cpp
  src/DataLoaderBridge.cpp
  src/MongoDBBridge.cpp
  src/TFListenerProxy.cpp
  src/Storage.cpp
  src/visualizer.cpp
  src/VisualizableGroupManager.cpp
  src/Visualizable.cpp)
target_link_libraries(rs_io
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${Boost_LIBRARIES}
  rs_core
  rs_utils
)

rs_add_library(rs_StorageWriter src/StorageWriter.cpp)
target_link_libraries(rs_StorageWriter rs_io)

rs_add_library(rs_CollectionReader src/CollectionReader.cpp)
target_link_libraries(rs_CollectionReader rs_core rs_io)

rs_add_library(rs_TFBroadcaster src/TFBroadcaster.cpp)
target_link_libraries(rs_TFBroadcaster rs_core)

rs_add_library(rs_DrawResultImage src/DrawResultImage.cpp)
target_link_libraries(rs_DrawResultImage rs_core)
