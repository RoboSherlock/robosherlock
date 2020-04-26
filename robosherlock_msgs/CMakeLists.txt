cmake_minimum_required(VERSION 2.8.3)
project(robosherlock_msgs)

find_package(catkin REQUIRED COMPONENTS 
  geometry_msgs 
  shape_msgs
  sensor_msgs
  visualization_msgs
  actionlib_msgs
  actionlib
  message_generation)

add_message_files(
  FILES
  RSObjectDescriptions.msg
  RSActiveAnnotatorList.msg
  ObjectDesignator.msg
)

add_service_files(FILES
  SetRSContext.srv
  RSQueryService.srv
  RSVisControl.srv
  UpdateObjects.srv
  ExecutePipeline.srv
  OverwriteParam.srv
  ReconfigureAnnotator.srv
)

add_action_files(DIRECTORY action FILES RSQuery.action)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  visualization_msgs
  actionlib_msgs
  shape_msgs
  sensor_msgs)

catkin_package(
  CATKIN_DEPENDS 
    geometry_msgs 
    shape_msgs 
    sensor_msgs
    message_runtime
)
