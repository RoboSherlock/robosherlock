project(rs_detection)

rs_add_library(rs_BoardDetector src/BoardDetector.cpp)
target_link_libraries(rs_BoardDetector rs_core)

rs_add_library(rs_PlaneAnnotator src/PlaneAnnotator.cpp)
target_link_libraries(rs_PlaneAnnotator rs_core)

install(
        TARGETS rs_BoardDetector
        rs_PlaneAnnotator
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
