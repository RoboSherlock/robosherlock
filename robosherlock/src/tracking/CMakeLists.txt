project(rs_tracking)

rs_add_library(rs_ObjectIdentityResolution src/ObjectIdentityResolution.cpp)
target_link_libraries(rs_ObjectIdentityResolution rs_core)

rs_add_library(rs_ClusterTracker src/ClusterTracker.cpp)
target_link_libraries(rs_ClusterTracker rs_core )

if(OpenCV_VERSION VERSION_GREATER "3.3")
    rs_add_library(rs_KCFTrackingAnnotator src/KCFTrackingAnnotator.cpp)
    target_link_libraries(rs_KCFTrackingAnnotator ${catkin_LIBRARIES})
endif()

rs_add_library(rs_PCLParticleTrackingAnnotator src/PCLParticleTrackingAnnotator.cpp)
target_link_libraries(rs_PCLParticleTrackingAnnotator ${catkin_LIBRARIES})


install(
        TARGETS rs_ObjectIdentityResolution
        rs_ClusterTracker
        rs_PCLParticleTrackingAnnotator
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)