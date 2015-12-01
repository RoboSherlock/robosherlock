#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/io/ROSCamInterface.h>

// Implementation

ROSCamInterface::ROSCamInterface(const boost::property_tree::ptree &pt)
  : CamInterface(pt), lookUpViewpoint(false), spinner(0), nodeHandle("~")
{
  listener = new tf::TransformListener(nodeHandle,ros::Duration(10.0));
  tfFrom = pt.get<std::string>("tf.from");
  tfTo = pt.get<std::string>("tf.to");
  lookUpViewpoint = pt.get<bool>("tf.lookupViewpoint");

  outInfo("TF Lookup: " FG_BLUE << (lookUpViewpoint ? "ON" : "OFF"));
  outInfo("TF From:   " FG_BLUE << tfFrom);
  outInfo("TF To:     " FG_BLUE << tfTo);
}

ROSCamInterface::~ROSCamInterface()
{
  delete listener;
}
