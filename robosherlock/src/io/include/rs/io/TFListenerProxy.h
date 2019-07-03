#ifndef TFLISTENERPROXY_H
#define TFLISTENERPROXY_H

#include <tf/transform_listener.h>

namespace rs
{
/**
 * @brief The TFListenerProxy class; because of the latched tf_static topic it is recommended to have a single
 * TFListener in a single node. Otherwise the buffer might not get filled correctly. Use this class with the static
 * listener member to lookup tf transforms in the annotators.
 */
class TFListenerProxy
{
public:
  static tf::TransformListener* listener;

  TFListenerProxy()
  {
    if (listener == nullptr)
    {
      listener = new tf::TransformListener();
    }
  }
  ~TFListenerProxy(){

  }
};
}  // namespace rs
#endif
