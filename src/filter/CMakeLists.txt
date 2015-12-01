project(rs_filter)

rs_add_library(rs_PointCloudFilter src/PointCloudFilter.cpp)
target_link_libraries(rs_PointCloudFilter rs_core)

rs_add_library(rs_RegionFilter src/RegionFilter.cpp)
target_link_libraries(rs_RegionFilter rs_core)

