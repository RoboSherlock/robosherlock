rs_add_library(rs_handleAnnotator 
        HandleAnnotator.cpp 
        HandleExtractor.cpp)
target_link_libraries(rs_handleAnnotator 
        ${catkin_LIBRARIES})

add_library(rs_templateMatcher 
        TemplateAlignment.cpp)
target_link_libraries(rs_templateMatcher 
        ${catkin_LIBRARIES})

rs_add_library(rs_templateAlignmentAnnotator 
        TemplateAlignmentAnnotator.cpp)
target_link_libraries(rs_templateAlignmentAnnotator 
        rs_templateMatcher 
        ${catkin_LIBRARIES})

rs_add_library(rs_clusterToPartsSegmenter 
        ClusterToPartsSegmenter.cpp)
target_link_libraries(rs_clusterToPartsSegmenter 
        ${catkin_LIBRARIES}
        ${SWIPL_LIBRARIES})

rs_add_library(rs_beliefToKnowRob 
	BeliefToKnowRob.cpp)
target_link_libraries(rs_beliefToKnowRob 
        ${catkin_LIBRARIES})

rs_add_library(rs_getRenderedViews 
	GetRenderedViews.cpp)
target_link_libraries(rs_getRenderedViews 
	${catkin_LIBRARIES})

rs_add_library(rs_offScreenSceneVariation 
	OffScreenSceneVariation.cpp)
target_link_libraries(rs_offScreenSceneVariation 
	${catkin_LIBRARIES})

rs_add_library(rs_unrealTableScene
        UnrealTableScene.cpp)
target_link_libraries(rs_unrealTableScene
        ${catkin_LIBRARIES})
