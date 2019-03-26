#include <rs/io/TFListenerProxy.h>

using namespace rs;

std::unique_ptr<tf::TransformListener> TFListenerProxy::listener = nullptr;
