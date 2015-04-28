#ifndef __LOCALIZATION_LOCALIZATION_H__
#define __LOCALIZATION_LOCALIZATION_H__

#include <ros/ros.h>

#include <rss_msgs/OdometryMsg.h>
#include <rss_msgs/RobotLocation.h>
#include <rss_msgs/SonarMsg.h>

#include "wall_map.h"

#include <memory>

namespace localization {

struct Particle {
  double x;
  double y;
  double t;
  double belief;

  Particle() {}
  Particle(double x_, double y_, double t_, double belief_) :
    x(x_), y(y_), t(t_), belief(belief_) {}
};

class Localization {
 public:
  Localization();

  /**
   * Method for updating current robot position. A message from Localization module.
   **/
  void onOdometryUpdate(const rss_msgs::OdometryMsg::ConstPtr &msg);
  void onSonarUpdate(const rss_msgs::SonarMsg::ConstPtr &msg);
  rss_msgs::RobotLocation currentPositionBelief() const;

 private:
  static const int N = 50000;
  // 5 cm distance tolerance
  static constexpr double DISTANCE_TOLERANCE = 0.05;
  // 10 degree heading tolerance
  static constexpr double HEADING_TOLERANCE = 0.17453292519;

  void InitializeParticles();
  void PublishLocation();
  void Test();

 private:
  ros::Publisher _guipoly_pub;
  ros::Publisher _location_pub;
  std::shared_ptr<WallMap> _wall_map;
  std::vector<Particle> _particles;
 
};

double NormalizeRad(double rad);

} // namespace localization 

#endif // __LOCALIZATION_LOCALIZATION_H__
