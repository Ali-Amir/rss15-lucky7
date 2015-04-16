#ifndef NAVIGATION_NAVIGATION_H
#define NAVIGATION_NAVIGATION_H

#include "ros/ros.h"

#include "rss_msgs/RobotLocation.h"

namespace navigation {

class Navigation {
 public:
  Navigation();

  /**
   * Method for updating current robot position. A message from Localization module.
   **/
  void updateLocation(const rss_msgs::RobotLocation::ConstPtr& msg);

  /**
   * Method for handling move commands (to specified location). Sends commands to motion control module.
   **/
  void moveRobotTo(const rss_msgs::RobotLocation::ConstPtr& msg);

 private:
};

} // namespace navigation

#endif
