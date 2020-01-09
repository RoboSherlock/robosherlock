project(rs_recognition)

add_library(rs_recognition SHARED src/LinemodInterface.cpp)
target_link_libraries(rs_recognition ${OpenCV_LIBRARIES})

rs_add_library(rs_LinemodAnnotator src/LinemodAnnotator.cpp)
target_link_libraries(rs_LinemodAnnotator rs_core rs_recognition)

if(LIBPRACMLN)
  rs_add_library(rs_MLNInferencer src/MLNInferencer.cpp)
  target_link_libraries(rs_MLNInferencer rs_core ${LIBPRACMLN})
endif(LIBPRACMLN)

if(Caffe_FOUND)
  add_library(rs_caffeProxy src/CaffeProxy.cpp)
  target_link_libraries(rs_caffeProxy ${catkin_LIBRARIES} ${Caffe_LIBRARIES})
  install(TARGETS rs_caffeProxy LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
  set(RS_CAFFE_LIB rs_caffeProxy)
endif(Caffe_FOUND)

install(
  TARGETS rs_recognition
          rs_LinemodAnnotator           
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
) 
