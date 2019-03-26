#include <rs/io/TFListenerProxy.h>

using namespace rs;

std::shared_ptr<tf::TransformListener> TFListenerProxy::listener = nullptr;
