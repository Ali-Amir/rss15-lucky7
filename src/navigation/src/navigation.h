#ifndef __NAVIGATION_NAVIGATION_H__
#define __NAVIGATION_NAVIGATION_H__

#include <ros/ros.h>

#include <rss_msgs/RobotLocation.h>

#include "obstacle_map.h"
#include "grid.h"

#include <memory>

namespace navigation {

class Navigation {
 public:
  static const int GRANULARITY = 600;

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
  void TestWheelVelocities();
  void SmoothePath(const std::vector<cgal_kernel::Point_3> &path,
                   std::vector<cgal_kernel::Point_3> *spath);


 private:
  ros::Publisher _guierase_pub;
  ros::Publisher _guipoint_pub;
  ros::Publisher _guipoly_pub;
  ros::Publisher _motor_pub;
  std::shared_ptr<cspace::ObstacleMap> _obs_map;
  std::unique_ptr<cspace::Grid> _world;
  rss_msgs::RobotLocation _cur_loc;
 
};

double NormalizeRad(double rad);

} // namespace navigation

#endif // __NAVIGATION_NAVIGATION_H__
