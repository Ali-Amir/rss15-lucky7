#include "localization.h"

#include "gui_msgs/GUIEraseMsg.h"
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
typedef CGAL::Polygon_2<K> Polygon_2;

namespace localization {

Localization::Localization() {
  ROS_INFO("Initializing localization module");

  ros::NodeHandle n;
  // Initialize message publishers.
  _location_pub = n.advertise<GUIEraseMsg>("localization/update", 1000);
  _guipoly_pub = n.advertise<GUIPolyMsg>("gui/Poly", 1000);

  // Load map file location and initialize the map-handler class instance.
  string mapfile_location;
  assert(n.getParam("/loc/mapFileName", mapfile_location));
  ROS_INFO("Got mapFileName: %s", mapfile_location.c_str());
  _wall_map = make_shared<WallMap>(mapfile_location);
  ROS_INFO("Initialized wall map.");

  InitializeParticles();
  Test();
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
    par.t = fmod(par.t, 2*M_PI);
    if (par.t < 0) {
      par.t += 2*M_PI;
    }
    t += par.t*prob;
    normalizer += prob;
  }

  x /= normalizer;
  y /= normalizer;
  t /= normalizer;
  t = fmod(t, 2*M_PI);
  if (t < 0) {
    t += 2*M_PI;
  }

  RobotLocation currentPosition;
  currentPosition.x = x;
  currentPosition.y = y;
  currentPosition.theta = t;
  return currentPosition;
}

void Localization::PublishLocation() {
  _location_pub.publish(currentPositionBelief());
}

void Localization::Test() {
  // TEST
  ROS_INFO("Distance to wall from 0.0 0.0 in direction 1.0 0.0: %.3lf",
           _wall_map->DistanceToWall(K::Ray_2(Point_2(0.0,0.0), K::Direction_2(1.0,1.00001))));
  // Initialize distribution:
  // TODO: REMOVE TEST CODE
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

  // Normalize the result
  double normalizer = 0.0;
  for (const Particle &par : _particles) {
    normalizer += exp(par.belief);
  }
  for (Particle &par : _particles) {
    par.belief -= log(normalizer);
  }

  PublishLocation();
}

void Localization::onSonarUpdate(const SonarMsg::ConstPtr &son) {
  PublishLocation();
}

/*
void Navigation::moveRobotTo(const RobotLocation::ConstPtr &target) {
  const auto &polys = _obs_map->_lvl_obstacles[0];
  for (const shared_ptr<Polygon_2> &poly : polys) {
    GUIPolyMsg new_poly;
    new_poly.numVertices = poly->size();
    for (int i = 0; i < poly->size(); ++i) {
      new_poly.x.push_back(CGAL::to_double((*poly)[i].x()));
      new_poly.y.push_back(CGAL::to_double((*poly)[i].y()));
    }
    new_poly.c.r = 255;
    new_poly.c.g = 0;
    new_poly.c.b = 0;
    new_poly.closed = 1;
    _guipoly_pub.publish(new_poly);
  }
  double cur_time = ros::Time::now().toSec();
  ROS_INFO("Got a moveRobotTo command at navigation module.");

  _world->ComputePathsToGoal(Point_2(target->x, target->y));
  ROS_INFO("Computed all paths. Now getting the specific one.");
  Grid::CellId cur_cell_id;
  assert(_world->GetCellId(
        Point_3(_cur_loc.x, _cur_loc.y,
                ObstacleMap::RadToRotation(_cur_loc.theta)),
        &cur_cell_id));
  const Grid::Cell *cur_cell(_world->GetCell(cur_cell_id));
  ROS_INFO("Search took: %.3lf sec.", ros::Time::now().toSec()-cur_time);
  ROS_ASSERT(cur_cell->HasPathToGoal());

  ROS_INFO("Current location: %.3lf %.3lf target: %.3lf %.3lf. Path length: %.3lf", _cur_loc.x, _cur_loc.y, target->x, target->y, cur_cell->min_dist_to_goal);

  vector<Point_3> path;
  path.push_back(Point_3(_cur_loc.x, _cur_loc.y, ObstacleMap::RadToRotation(_cur_loc.theta)));
  for (; cur_cell->to_goal_next != nullptr; cur_cell = cur_cell->to_goal_next) {
    //ROS_INFO("Point: %.3lf %.3lf angle: %.3lf", cur_cell->xc, cur_cell->yc, ObstacleMap::IdToRotation(cur_cell->rotId));
    path.push_back(Point_3(cur_cell->xc, cur_cell->yc,
                           ObstacleMap::IdToRotation(cur_cell->rotId)));
  }
  path.push_back(Point_3(cur_cell->xc, cur_cell->yc,
                         ObstacleMap::IdToRotation(cur_cell->rotId)));
  vector<Point_3> smooth_path;
  SmoothePath(path, &smooth_path);
  // TODO: Smoothen the path, etc.
  ROS_INFO("Publishing the path of size: %d", int(smooth_path.size()));

  GUIPolyMsg path_poly;
  path_poly.numVertices = smooth_path.size();
  for (const Point_3 &p : smooth_path) {
    path_poly.x.push_back(CGAL::to_double(p.x()));
    path_poly.y.push_back(CGAL::to_double(p.y()));
  }
  path_poly.c.r = 255;
  path_poly.c.g = 0;
  path_poly.c.b = 0;
  _guipoly_pub.publish(path_poly);
}

void Navigation::SmoothePath(const vector<Point_3> &path,
                             vector<Point_3> *spath) {
  spath->clear();
  for (int st = 0, next_start; st < (int)path.size()-1; st = next_start) {
    int j = path.size()-1;//min(st+50, (int)path.size() - 1);
    vector<Point_3> smooth;
    for (; j > st+1; --j) {
      ROS_INFO_THROTTLE(5, "Here for st=%d j=%d", st, j);
      double stax = CGAL::to_double(path[st].x());
      double stay = CGAL::to_double(path[st].y());
      double tarx = CGAL::to_double(path[j].x());
      double tary = CGAL::to_double(path[j].y());
      double d = sqrt((stax-tarx)*(stax-tarx)+(stay-tary)*(stay-tary));

      if (d < 1e-8) {
        break;
      }

      double heading_rad = atan2(tary-stay, tarx-stax);

      double sta_rad = NormalizeRad(ObstacleMap::RotationToRad(CGAL::to_double(path[st].z())));
      double tar_rad = NormalizeRad(heading_rad-(sta_rad-heading_rad));
      if (sta_rad < tar_rad && tar_rad-sta_rad > M_PI) {
        sta_rad += 2.0*M_PI;
      }
      if (sta_rad > tar_rad && sta_rad-tar_rad > M_PI) {
        tar_rad += 2.0*M_PI;
      }

      double dir;
      if (cos(heading_rad-sta_rad) < 0.0) {
        dir = -1.0;
      } else {
        dir = 1.0;
      }

      bool is_straight = fabs(sin(heading_rad-sta_rad)) < 1e-6;
      double mul;
      if (fabs(sin(heading_rad-sta_rad)) < 1e-6) {
        mul = d;
      } else {
        double radius = d / 2.0 / abs(sin(heading_rad-sta_rad));
        mul = fabs(sta_rad-tar_rad)*radius;
      }
      double radius = is_straight ? 0.0 : (d / 2.0 / abs(sin(heading_rad-sta_rad)));

      double curx = stax;
      double cury = stay;
      double cur_rad = sta_rad;

      bool ok = 1;
      for (int step = 0; step < GRANULARITY; ++step) {
        Point_3 cur_point(curx, cury, ObstacleMap::RadToRotation(cur_rad));
        Grid::CellId cur_cell;
        if (!_world->GetCellId(cur_point, &cur_cell)) {
          ok = false;
          break;
        }
        if (_world->GetCell(cur_cell) == nullptr) {
          ok = false;
          break;
        }
        curx = curx + dir*mul*cos(cur_rad)/GRANULARITY;
        cury = cury + dir*mul*sin(cur_rad)/GRANULARITY;
        cur_rad = cur_rad + (tar_rad-sta_rad)/GRANULARITY;
        smooth.push_back(cur_point);
      }
      if (ok) {
        break;
      }
      smooth.clear();
    }
    if (!smooth.size()) {
      for (int k = st; k < j; ++k) {
        smooth.push_back(path[k]);
      }
    }
    spath->insert(spath->end(), smooth.begin(), smooth.end());
    next_start = j;
  }
  spath->push_back(path.back());
}
*/

double NormalizeRad(double rad) {
  rad = fmod(rad, 2*M_PI);
  if (rad < 0) {
    rad += 2*M_PI;
  }
  return rad;
}

} // namespace navigation
