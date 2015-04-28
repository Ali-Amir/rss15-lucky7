#ifndef __LOCALIZATION_LOCALIZATION_H__
#define __LOCALIZATION_LOCALIZATION_H__

#include <ros/ros.h>

#include <rss_msgs/OdometryMsg.h>
#include <rss_msgs/SonarMsg.h>

#include "wall_map.h"

#include <memory>

namespace localization {

class Localization {
 public:
  Localization();

  /**
   * Method for updating current robot position. A message from Localization module.
   **/
  void onOdometryUpdate(const rss_msgs::OdometryMsg::ConstPtr &msg);
  void onSonarUpdate(const rss_msgs::SonarMsg::ConstPtr &msg);

 private:


 private:
  ros::Publisher _guipoly_pub;
  ros::Publisher _location_pub;
  std::shared_ptr<WallMap> _wall_map;
 
};

double NormalizeRad(double rad);

} // namespace localization 

#endif // __LOCALIZATION_LOCALIZATION_H__
