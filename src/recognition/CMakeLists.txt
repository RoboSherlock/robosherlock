project(rs_recognition)

add_library(rs_recognition SHARED
  src/LinemodInterface.cpp
)
target_link_libraries(rs_recognition ${OpenCV_LIBRARIES})

rs_add_library(rs_ClassifierDetection src/ClassifierDetection.cpp)
target_link_libraries(rs_ClassifierDetection rs_core)

rs_add_library(rs_LinemodAnnotator src/LinemodAnnotator.cpp)
target_link_libraries(rs_LinemodAnnotator rs_core rs_recognition)

rs_add_library(rs_KNNClassifier src/KNNClassifier.cpp)
target_link_libraries(rs_KNNClassifier rs_core)

if(LIBPRACMLN)
  rs_add_library(rs_MLNInferencer src/MLNInferencer.cpp)
  target_link_libraries(rs_MLNInferencer rs_core ${LIBPRACMLN})
endif(LIBPRACMLN)

if(Caffe_FOUND)
    rs_add_library(rs_caffeProxy src/CaffeProxy.cpp)
    target_link_libraries(rs_caffeProxy ${catkin_LIBRARIES} ${Caffe_LIBRARIES})
endif(Caffe_FOUND)
