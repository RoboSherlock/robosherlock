project(rs_filter)

rs_add_library(rs_PointCloudFilter src/PointCloudFilter.cpp)
target_link_libraries(rs_PointCloudFilter rs_core rs_io)

rs_add_library(rs_RegionFilter src/RegionFilter.cpp)
target_link_libraries(rs_RegionFilter ${YAML_CPP_LIBRARIES} rs_core)

rs_add_library(rs_ClusterFilter src/ClusterFilter.cpp)
target_link_libraries(rs_ClusterFilter rs_core)


install(
        TARGETS rs_PointCloudFilter
        rs_RegionFilter
        rs_ClusterFilter
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)