#include "localization.h"

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

Localization::Localization() {
  ROS_INFO("Initializing localization module");

  // Initializing the sonar locations.
  SONAR_DIR = {
    cgal_kernel::Vector_2(0.0, 1.0),
    cgal_kernel::Vector_2(0.0, 1.0)
  };
  SONAR_POS = {
    cgal_kernel::Vector_2(-0.28, 0.215),
    cgal_kernel::Vector_2(0.025, 0.215)
  };

  ros::NodeHandle n;
  // Initialize message publishers.
  _location_pub = n.advertise<RobotLocation>("localization/update", 1000);
  _guipoly_pub = n.advertise<GUIPolyMsg>("gui/Poly", 1000);

  // Load map file location and initialize the map-handler class instance.
  string mapfile_location;
  assert(n.getParam("/loc/mapFileName", mapfile_location));
  ROS_INFO("Got mapFileName: %s", mapfile_location.c_str());
  _wall_map = make_shared<WallMap>(mapfile_location);
  ROS_INFO("Initialized wall map.");

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

  default_random_engine generator;
  uniform_real_distribution<double> xDist(sx-dx, sx+dx);
  uniform_real_distribution<double> yDist(sy-dy, sy+dy);
  uniform_real_distribution<double> tDist(st-dt, st+dt);

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
  for (auto par : _particles) {
    double prob = exp(par.belief);
    x += par.x*prob;
    y += par.y*prob;
    par.t = NormalizeRad(par.t);
    t += par.t*prob;
    normalizer += prob;
  }

  x /= normalizer;
  y /= normalizer;
  t /= normalizer;
  t = NormalizeRad(t);

  RobotLocation currentPosition;
  currentPosition.x = x;
  currentPosition.y = y;
  currentPosition.theta = t;
  return currentPosition;
}

void Localization::PublishLocation() {
  _location_pub.publish(currentPositionBelief());
}

void Localization::TestTriangulation() {
  // TEST
  ROS_INFO("Distance to wall from 0.0 0.0 in direction 1.0 0.0: %.3lf",
           _wall_map->DistanceToWall(K::Ray_2(Point_2(0.0,0.0), K::Direction_2(1.0,1.00001))));
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
  return -0.5*log(2.0*M_PI*var) - ((x-mux)*(x-mux)+(y-muy)*(y-muy))/var/2.0;
}

void Localization::NormalizeBeliefs() {
  // Normalize the result
  double normalizer = 0.0;
  for (const Particle &par : _particles) {
    normalizer += exp(par.belief);
  }
  for (Particle &par : _particles) {
    par.belief -= log(normalizer);
  }
}

void Localization::onOdometryUpdate(const OdometryMsg::ConstPtr &odo) {
  double dx = odo->x;
  double dy = odo->y;
  double dt = odo->theta;

  double varD = pow(min(0.03, 0.01 + sqrt(dx*dx+dy*dy)), 2.0);
  double varT = pow(min(0.17453292519, 0.17453292519/20.0+fabs(dt)), 2.0);

  default_random_engine gen;
  normal_distribution<double> xyDist(0.0, varD);
  normal_distribution<double> tDist(0.0, varT);

  set<Particle> new_particles;
  for (const Particle &par : _particles) {
    new_particles.insert(Particle(par.x+dx, par.y+dy, par.t+dt,
          par.belief + logGaussian(0.0, 0.0, 0.0, 0.0, varD) + logGaussian(0.0, 0.0, varT)));
    // Generate 10 random points to account for noise.
    for (int i = 0; i < 10; ++i) {
      double x = xyDist(gen) + par.x+dx;
      double y = xyDist(gen) + par.y+dy;
      double t = tDist(gen) + par.t+dt;
      new_particles.insert(Particle(x, y, t,
            par.belief + logGaussian(x-par.x-dx, 0.0, y-par.y-dy, 0.0, varD) + logGaussian(t-par.t-dt, 0.0, varT)));
    }
  }

  // Substitute old particles with top #N new ones.
  for (; new_particles.size() > N; new_particles.erase(new_particles.begin()));
  _particles.clear();
  copy(new_particles.begin(), new_particles.end(), back_inserter(_particles));

  NormalizeBeliefs();
  PublishLocation();
}

Vector_2 Rotate(Vector_2 vec, double alfa) {
  return Vector_2(vec.x()*cos(alfa) - vec.y()*sin(alfa),
                  vec.x()*sin(alfa) + vec.y()*cos(alfa));
}

void Localization::onSonarUpdate(const SonarMsg::ConstPtr &son) {
  double varD = 0.01;
  for (Particle &par : _particles) {
    Vector_2 dir(Rotate(SONAR_DIR[son->sonarId], par.t));
    Point_2 sonarLoc(Point_2(par.x, par.y) + Rotate(SONAR_POS[son->sonarId], par.t));
    double mu = _wall_map->DistanceToWall(Ray_2(sonarLoc, dir));
    par.belief += logGaussian(son->range, mu, varD);
  }
  NormalizeBeliefs();
  PublishLocation();
}

double NormalizeRad(double rad) {
  rad = fmod(rad, 2*M_PI);
  if (rad < 0) {
    rad += 2*M_PI;
  }
  return rad;
}

} // namespace navigation
