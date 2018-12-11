/* 
* Roboception GmbH 
* Munich, Germany 
* www.roboception.com 
* 
* Copyright (c) 2018 Roboception GmbH 
* All rights reserved 
* 
* Author: Raphael Schaller
*/
#ifndef __ROS_RCVISARD_BRIDGE_H__
#define __ROS_RCVISARD_BRIDGE_H__

// ROS
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

// RS
#include <rs/io/ROSCamInterface.h>

class ROSRcVisardBridge : public ROSCamInterface
{
  private:
    void initSpinner();
    void readConfig(const boost::property_tree::ptree &pt);

    template<typename T, int tuple_pos>
    void cb(const boost::shared_ptr<const T> &msg);

    void fillSubscriberBitMaps();

  private:
    enum class ProjectorMode
    {
        DEFAULT,
        EXPOSURE_ALTERNATE
    };

    enum
    {
      LEFT_RECT_POS = 0,
      RIGHT_RECT_POS,
      LEFT_CAM_INFO_POS,
      RIGHT_CAM_INFO_POS,
      DEPTH_POS,
      DISPARITY_ERROR_POS,
      CONFIDENCE_POS,
      NUM_TOPICS
    };

    using DataSet = std::tuple<
        sensor_msgs::ImageConstPtr,
        sensor_msgs::ImageConstPtr,
        sensor_msgs::CameraInfoConstPtr,
        sensor_msgs::CameraInfoConstPtr,
        sensor_msgs::ImageConstPtr,
        sensor_msgs::ImageConstPtr,
        sensor_msgs::ImageConstPtr>;

    static constexpr std::array<bool, NUM_TOPICS> exposure_alternate_topics_first_
        {
            {false, false, true, true, true, true, true}
        };
    static constexpr std::array<bool, NUM_TOPICS> exposure_alternate_topics_second_
        {
            {true, true, true, true, false, false, false}
        };

    int max_queue_size_ = 50;

    ProjectorMode current_projector_mode_;

    std::array<ros::Subscriber, NUM_TOPICS> subs_;
    std::array<bool, NUM_TOPICS> sub_bit_map_first_;
    std::array<bool, NUM_TOPICS> sub_bit_map_second_;

    using DataSetMap = std::map<ros::Time, DataSet>;
    DataSetMap data_queue_;
    DataSet current_data_set_;

    std::mutex cb_mutex_;

  public:
    ROSRcVisardBridge(const boost::property_tree::ptree &pt);
    ~ROSRcVisardBridge();

  protected:
    virtual bool setData(uima::CAS &tcas, uint64_t ts) override;
};

#endif //__ROS_RCVISARD_BRIDGE_H__
