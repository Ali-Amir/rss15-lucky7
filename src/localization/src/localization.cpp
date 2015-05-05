#include "localization.h"

#include "gui_msgs/ColorMsg.h"
#include "gui_msgs/GUIPointMsg.h"
#include "gui_msgs/GUIPolyMsg.h"
#include "rss_msgs/MotionMsg.h"
#include "rss_msgs/RobotLocation.h"
#include <random>

using namespace rss_msgs;
using namespace gui_msgs;
using namespace std;

typedef cgal_kernel K;
typedef K::Point_2 Point_2;
typedef K::Point_3 Point_3;
typedef K::Ray_2 Ray_2;
typedef K::Vector_2 Vector_2;
typedef CGAL::Polygon_2<K> Polygon_2;

namespace localization {

double curTime() {
  return ros::Time::now().toSec();
}

Localization::Localization() {
  ROS_DEBUG("Initializing localization module");

  _time = curTime();

  // Initializing the sonar locations.
  SONAR_DIR = {
    cgal_kernel::Vector_2(0.0, -1.0),
    cgal_kernel::Vector_2(0.0, 1.0)
  };
  SONAR_POS = {
    cgal_kernel::Vector_2(-0.0975, -0.215),
    cgal_kernel::Vector_2(0.025, 0.215)
  };

  ros::NodeHandle n;
  // Initialize message publishers.
  _location_pub = n.advertise<RobotLocation>("localization/update", 1000);
  _guipoly_pub = n.advertise<GUIPolyMsg>("gui/Poly", 1000);
  _guipoint_pub = n.advertise<GUIPointMsg>("gui/Point", 1000);

  // Load map file location and initialize the map-handler class instance.
  string mapfile_location;
  assert(n.getParam("/loc/mapFileName", mapfile_location));
  ROS_DEBUG("Got mapFileName: %s", mapfile_location.c_str());
  _wall_map = make_shared<WallMap>(mapfile_location);
  ROS_DEBUG("Initialized wall map.");

  InitializeParticles();

  // Test CASE 01
  {
    string res;
    if (n.getParam("/loc/testTriangulation", res)) {
      if (res == "yes") {
        TestTriangulation();
      }
    }
  }
  // Test CASE 02
  {
    string res;
    if (n.getParam("/loc/leaveBreadCrumbs", res)) {
      if (res == "yes") {
        _leaveBreadCrumbs = true;
      }
    }
  }

  /*
  boost::shared_ptr<SonarMsg> son(
    new SonarMsg());
  onSonarUpdate(son);
  */
}

// Initialize with N random points generated within a given tolerance (in 3-D
// space, it's a parallelipiped with sides equal to tolerance values)
void Localization::InitializeParticles() {
  _particles.resize(N); 

  double sx = CGAL::to_double(_wall_map->GetRobotStart().x());
  double dx = DISTANCE_TOLERANCE;
  double sy = CGAL::to_double(_wall_map->GetRobotStart().y());
  double dy = DISTANCE_TOLERANCE;
  double st = 0.0;
  double dt = HEADING_TOLERANCE;
  _prev_odo_x = sx;
  _prev_odo_y = sy;
  _prev_odo_t = 0.0;
  ROS_INFO("Init x: %.3lf y: %.3lf", sx, sy);

  default_random_engine generator;
  uniform_real_distribution<double> xDist(sx-dx, sx+dx);
  uniform_real_distribution<double> yDist(sy-dy, sy+dy);
  uniform_real_distribution<double> tDist(st-dt, st+dt);

  /*
  uniform_real_distribution<double> xDist(sx-dx, sx+dx);
  uniform_real_distribution<double> yDist(sy-dy, sy+dy);
  uniform_real_distribution<double> tDist(0.0, 2*M_PI);
  */

  for (int i = 0; i < N; ++i) {
    double x = xDist(generator);
    double y = yDist(generator);
    double t = tDist(generator);
    _particles[i] = Particle(x, y, t, -log(1.0*N));
  }
}

RobotLocation Localization::currentPositionBelief() const {
  double x = 0, y = 0, t = 0;
  double normalizer = 0.0;
  double maxBelief = -1e18;
  for (auto par : _particles) {
    if (par.belief > maxBelief) {
      maxBelief = par.belief;
      x = par.x;
      y = par.y;
      t = par.t;
    }
    /*
    double prob = exp(par.belief);
    x += par.x*prob;
    y += par.y*prob;
    par.t = NormalizeRad(par.t);
    t += par.t*prob;
    normalizer += prob;
    */
  }

  /*
  x /= normalizer;
  y /= normalizer;
  t /= normalizer;
  t = NormalizeRad(t);
  */

  RobotLocation currentPosition;
  currentPosition.x = x;
  currentPosition.y = y;
  currentPosition.theta = NormalizeRad(t);
  //ROS_INFO("Belief is: %.3lf", exp(maxBelief));
  return currentPosition;
}

void Localization::PublishLocation() {
  RobotLocation currentBelief = currentPositionBelief();
  ROS_INFO_THROTTLE(2,
      "Current belief: (%.3lf %.3lf|%.3lf) Odometry: (%.3lf %.3lf|%.3lf)\n",
      currentBelief.x, currentBelief.y, currentBelief.theta,
      _prev_odo_x, _prev_odo_y, _prev_odo_t);
  // TODO: remove
  /*
  currentBelief.x = _prev_odo_x;
  currentBelief.y = _prev_odo_y;
  currentBelief.theta = _prev_odo_t;
  */
  _location_pub.publish(currentBelief);

  if (_leaveBreadCrumbs) {
    GUIPointMsg crumb;
    ColorMsg color;
    color.r = 0;
    color.g = 255;
    color.b = 0;
    crumb.color = color;
    crumb.x = currentBelief.x;
    crumb.y = currentBelief.y;
    _guipoint_pub.publish(crumb);
  }
}

void Localization::TestTriangulation() {
  // TEST
  ROS_DEBUG("Distance to wall from 0.0 0.0 in direction 1.0 0.0: %.3lf",
           _wall_map->DistanceToWall(K::Ray_2(Point_2(0.0,0.0), K::Direction_2(1.0,1.00000))));
  // Initialize distribution:
  ros::Duration(5.0).sleep(); // Sleep for 5 sec to allow gui node to initialize.
  for (const K::Triangle_2 &tri : _wall_map->_triangles) {
    GUIPolyMsg new_poly;
    new_poly.numVertices = 3;
    for (int i = 0; i < 3; ++i) {
      new_poly.x.push_back(CGAL::to_double(tri[i].x()));
      new_poly.y.push_back(CGAL::to_double(tri[i].y()));
    }
    new_poly.c.r = 255;
    new_poly.c.g = 0;
    new_poly.c.b = 0;
    new_poly.closed = 1;
    _guipoly_pub.publish(new_poly);
  }
}

double logGaussian(double x, double mu, double var) {
  return -0.5*log(2.0*M_PI*var) - (x-mu)*(x-mu)/var/2.0;
}
double logGaussian(double x, double y, double mux, double muy, double var) {
  return -1.0*log(2.0*M_PI*var) - ((x-mux)*(x-mux)+(y-muy)*(y-muy))/var/2.0;
}

void Localization::NormalizeBeliefs() {
  // Normalize the result
  double normalizer = -1e18;
  for (const Particle &par : _particles) {
    normalizer = max(normalizer, par.belief);
    //normalizer += exp(par.belief);
  }
  for (Particle &par : _particles) {
    par.belief -= normalizer;
  }
}

void Localization::onOdometryUpdate(const OdometryMsg::ConstPtr &odo) {
  double dx = odo->x-_prev_odo_x;
  double dy = odo->y-_prev_odo_y;
  double dt = odo->theta-_prev_odo_t;
  if (fabs(dx) < 1e-3 && fabs(dy) < 1e-3 && fabs(dt) < M_PI*0.1/180) {
    PublishLocation();
    return;
  }

  _prev_odo_x = odo->x;
  _prev_odo_y = odo->y;
  _prev_odo_t = odo->theta;
  _prev_odo_time = curTime();

  double start_time = curTime();
  ROS_DEBUG_STREAM("onOdometryUpdate: " << dx << " " << dy <<
      " dt: " << dt << " at time: " << curTime()-_time);

  double varD = pow(0.001, 2.0);
  double varT = pow(0.017453293, 2.0);

  default_random_engine gen;
  normal_distribution<double> xyDist(0.0, varD);
  normal_distribution<double> tDist(0.0, varT);

  vector<Particle> new_particles;
  for (const Particle &par : _particles) {
    new_particles.push_back(Particle(par.x+dx, par.y+dy, par.t+dt,
          par.belief + logGaussian(0.0, 0.0, 0.0, 0.0, varD) +
          logGaussian(0.0, 0.0, varT)));
    // Generate 10 random points to account for noise.
    for (int i = 0; i < 10; ++i) {
      double x = xyDist(gen) + par.x+dx;
      double y = xyDist(gen) + par.y+dy;
      double t = tDist(gen) + par.t+dt;
      new_particles.push_back(Particle(x, y, t,
            par.belief + logGaussian(x-par.x-dx, 0.0, y-par.y-dy, 0.0, varD) + logGaussian(t-par.t-dt, 0.0, varT)));
    }
  }

  ROS_DEBUG("onOdometryUpdate: time up to generation %.3lf sec.", curTime()-start_time);
  // Substitute old particles with top #N new ones.
  sort(new_particles.begin(), new_particles.end());
  reverse(new_particles.begin(), new_particles.end());
  new_particles.resize(N);
  _particles = new_particles;
  ROS_DEBUG("onOdometryUpdate: time up to sorting %.3lf sec.", curTime()-start_time);

  NormalizeBeliefs();
  PublishLocation();
  ROS_DEBUG("onOdometryUpdate: time passed %.3lf sec.", curTime()-start_time);
}

Vector_2 Rotate(Vector_2 vec, double alfa) {
  return Vector_2(vec.x()*cos(alfa) - vec.y()*sin(alfa),
                  vec.x()*sin(alfa) + vec.y()*cos(alfa));
}

void Localization::onSonarUpdate(const SonarMsg::ConstPtr &son) {
  // Skip if sonar is out of range.
  if (son->range < 0.2 || son->range > 1.8) {
    return;
  }
  ROS_INFO_STREAM_THROTTLE(2,
      "Got sonar update: " << son->sonarId << " range: " << son->range);
  double start_time = curTime();
  double varD = 1.0;
  for (Particle &par : _particles) {
    Vector_2 dir(Rotate(SONAR_DIR[son->sonarId], par.t));
    Point_2 sonarLoc(Point_2(par.x, par.y) + Rotate(SONAR_POS[son->sonarId], par.t));
    double mu = _wall_map->DistanceToWall(Ray_2(sonarLoc, dir));
    par.belief += logGaussian(son->range, mu, varD);
  }
  NormalizeBeliefs();
  PublishLocation();
  ROS_DEBUG("onSonarUpdate: time passed %.3lf sec.", curTime()-start_time);
}

double NormalizeRad(double rad) {
  rad = fmod(rad, 2*M_PI);
  if (rad < 0) {
    rad += 2*M_PI;
  }
  return rad;
}

} // namespace navigation
