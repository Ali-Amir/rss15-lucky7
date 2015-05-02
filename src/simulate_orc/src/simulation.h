#ifndef __SIMULATION_SIMULATOR_H__
#define __SIMULATION_SIMULATOR_H__

#include <ros/ros.h>

#include <rss_msgs/MotionMsg.h>

#include <memory>

namespace simulation {

class Simulator {
 public:
  Simulator();

  void simulateMotion(const rss_msgs::MotionMsg::ConstPtr& msg);
  void step(double t);

 public:
  double tv{0.0};
  double rv{0.0};
  double x;
  double y;
  double theta;
 
};

} // namespace navigation

#endif // __NAVIGATION_NAVIGATION_H__
