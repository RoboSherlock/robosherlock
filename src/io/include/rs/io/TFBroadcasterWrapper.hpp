#ifndef TFBROADCASTERWRAPPER_HPP
#define TFBROADCASTERWRAPPER_HPP

#include <vector>
#include <thread>
#include <chrono>
#include <mutex>
#include <tf/transform_broadcaster.h>

/**
 * @brief The TFBroadcasterWrapper class
 * \description: helper class for broadcasting TF transforms from within annotators;
 * needs to be running on it's own thread;
 * TFBroadcasterWrapper broadcasterObject;
 * thread = std::thread(&BroadcasterWrapper::run, &broadCasterObject);
 * broadcasterObject->addTransforms(std::vector<td::StampedTransform>);
 */
class TFBroadcasterWrapper
{
private:
  std::vector<tf::StampedTransform> transforms;
  std::chrono::milliseconds sleepTime;

  bool volatile terminate_flag = false;
public:
  std::mutex mutex;

  TFBroadcasterWrapper(): transforms()
  {
    sleepTime = std::chrono::milliseconds(1000 / 30);
  }

  ~TFBroadcasterWrapper()
  {

  }

  /**
   * @brief run loop the thread to publish transforms
   */
  void run()
  {
    tf::TransformBroadcaster br;
    while(ros::ok() && !terminate_flag)
    {
      mutex.lock();
      if(!transforms.empty())
      {
        ros::Time t = ros::Time::now();
        for(int i = 0; i < transforms.size(); ++i)
        {
          transforms[i].stamp_ = t;
        }
        br.sendTransform(transforms);
      }
      mutex.unlock();
      std::this_thread::sleep_for(sleepTime);
    }
  }
  /**
   * @brief addTransforms add multiple transformations at once
   * @param ts vector of stamped transforms
   */
  void addTransforms(const std::vector<tf::StampedTransform> &ts)
  {
    mutex.lock();
    transforms.clear();
    transforms.insert(transforms.end(), ts.begin(), ts.end());
    mutex.unlock();
  }

  /**
   * @brief add a single transformation
   * @param ts StampedTransform to be added
   */
  void addTransform(tf::StampedTransform &ts)
  {
    mutex.lock();
    transforms.push_back(ts);
    mutex.unlock();
  }

  /**
   * @brief clear the buffer
   */
  void clear()
  {
    mutex.lock();
    transforms.clear();
    mutex.unlock();
  }

  void terminate()
  {
    terminate_flag = true;
  }
};

#endif // TFBROADCASTERWRAPPER_HPP
