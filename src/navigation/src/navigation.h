#ifndef __NAVIGATION_NAVIGATION_H__
#define __NAVIGATION_NAVIGATION_H__

#include "ros/ros.h"

#include "rss_msgs/RobotLocation.h"

#include "obstacle_map.h"
#include "grid.h"

#include <memory>

namespace navigation {

class Navigation {
 public:
  Navigation();

  /**
   * Method for updating current robot position. A message from Localization module.
   **/
  void updateRobotLocation(const rss_msgs::RobotLocation::ConstPtr& msg);

  /**
   * Method for handling move commands (to specified location). Sends commands to motion control module.
   **/
  void moveRobotTo(const rss_msgs::RobotLocation::ConstPtr& msg);

 private:
  ros::Publisher _guierase_pub;
  ros::Publisher _guipoint_pub;
  ros::Publisher _guipoly_pub;
  ros::Publisher _motor_pub;
  std::shared_ptr<cspace::ObstacleMap> _obs_map;
  std::unique_ptr<cspace::Grid> _world;
  rss_msgs::RobotLocation _cur_loc;
 
};

} // namespace navigation

#endif // __NAVIGATION_NAVIGATION_H__
