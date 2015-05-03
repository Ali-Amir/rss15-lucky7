#include "simulation.h"

#include "rss_msgs/MotionMsg.h"
#include "rss_msgs/RobotLocation.h"

using namespace rss_msgs;
using namespace std;

namespace simulation {

double CurTime() {
  return ros::Time::now().toSec();
}

Simulator::Simulator() {
  x = y = 0.6;
  theta = 0.0;
}

void Simulator::simulateMotion(const MotionMsg::ConstPtr &mot) {
  tv = mot->translationalVelocity;
  rv = mot->rotationalVelocity;
}

void Simulator::step(double t) {
  x += cos(theta)*tv*t;
  y += sin(theta)*tv*t;
  theta += rv*t;
  theta = fmod(theta, 2.0*M_PI);
  if (theta < 0.0) {
    theta += 2.0*M_PI;
  }
  if (theta > M_PI) {
    theta -= 2.0*M_PI;
  }
}

} // namespace navigation
