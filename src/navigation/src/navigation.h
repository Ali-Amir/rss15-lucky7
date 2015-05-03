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
  static constexpr double MAX_BLIND_TIME = 4.0;
  static constexpr double MAX_TRANS_VELOCITY = 0.15;
  static constexpr double MAX_ROT_VELOCITY = 0.157079633; // pi/4 / 5 sec
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
  void TestLocalization();
  void GetSmoothPathVelocities(const std::vector<cgal_kernel::Point_3> &path);
  void CapVelocities();
  void PublishGUICSObstacles(int lvl);
  bool UsePreviousCommand();


 private:
  ros::Publisher _guierase_pub;
  ros::Publisher _guipoint_pub;
  ros::Publisher _guipoly_pub;
  ros::Publisher _motor_pub;
  std::unique_ptr<cspace::Grid> _world;

  double _time;
  double _tmp_time;
  double _time_until{-1.0};
  bool _testLocalization{false};
  double _trans_velocity;
  double _rot_velocity;
  int _prevLevel{-1};
  double _time_paint{-1.0};
 
 public:
  std::shared_ptr<cspace::ObstacleMap> _obs_map;
  rss_msgs::RobotLocation _cur_loc;
};

double NormalizeRad(double rad);

} // namespace navigation

#endif // __NAVIGATION_NAVIGATION_H__
