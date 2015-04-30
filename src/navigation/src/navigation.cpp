#include "navigation.h"

#include "gui_msgs/GUIEraseMsg.h"
#include "gui_msgs/GUIPointMsg.h"
#include "gui_msgs/GUIPolyMsg.h"
#include "rss_msgs/MotionMsg.h"
#include "rss_msgs/RobotLocation.h"

using namespace rss_msgs;
using namespace gui_msgs;
using namespace cspace;
using namespace std;

typedef cgal_kernel K;
typedef K::Point_2 Point_2;
typedef K::Point_3 Point_3;
typedef CGAL::Polygon_2<K> Polygon_2;

namespace navigation {

Navigation::Navigation() {
  ROS_INFO("Initializing navigation module");

  _time = ros::Time::now().toSec();

  ros::NodeHandle n;
  // Initialize message publishers.
  _guierase_pub = n.advertise<GUIEraseMsg>("gui/Erase", 1000);
  _guipoint_pub = n.advertise<GUIPointMsg>("gui/Point", 1000);
  _guipoly_pub = n.advertise<GUIPolyMsg>("gui/Poly", 1000);
  _motor_pub = n.advertise<MotionMsg>("command/Motors", 1000);

  // Load map file location and initialize the map-handler class instance.
  string mapfile_location;
  assert(n.getParam("/nav/mapFileName", mapfile_location));
  ROS_INFO("Got mapFileName: %s", mapfile_location.c_str());
  _obs_map = make_shared<ObstacleMap>(mapfile_location);
  ROS_INFO("Initialized obstacle map.");

  // Initialize the grid graph.
  _world.reset(new Grid(_obs_map)); // RandomNet?
  ROS_INFO("Initialized world grid.");

  // TODO: Add localization subscriber.
  // TODO: remove
  boost::shared_ptr<RobotLocation> loc(new RobotLocation());
  loc->x = CGAL::to_double(_obs_map->_robot_goal.x());
  loc->y = CGAL::to_double(_obs_map->_robot_goal.y());
  loc->theta = 0.0;
  moveRobotTo(loc);

  // TODO: remove the test call
  // TEST CASE 01
  {
    string res;
    if (n.getParam("/nav/testWheelVelocities", res)) {
      if (res == "yes") {
        TestWheelVelocities();
      }
    }
  }
  // TEST CASE 02
  {
    string res;
    if (n.getParam("/nav/testLocalization", res)) {
      if (res == "yes") {
        _testLocalization = true;
      }
    }
  }
}

void Navigation::TestWheelVelocities() {
  double start_time = ros::Time::now().toSec();
  double cur_time;
  while ((cur_time = ros::Time::now().toSec()) - start_time < 10.0) {
    MotionMsg comm;
    comm.translationalVelocity = 0.2;
    comm.rotationalVelocity = 0.0;
    _motor_pub.publish(comm);
  }
  ROS_INFO("Time taken: %.3lf\n", cur_time-start_time);
  MotionMsg comm;
  comm.translationalVelocity = 0.0;
  comm.rotationalVelocity = 0.0;
  _motor_pub.publish(comm);
}

void Navigation::updateRobotLocation(const RobotLocation::ConstPtr &loc) {
  double cur_time = ros::Time::now().toSec();
  ROS_INFO("GOT POSITION UPDATE! TIME: %.3lf\n", cur_time-_time);
  _cur_loc = *loc;
  if (_testLocalization) {
    MotionMsg comm;
    comm.translationalVelocity = 0.05;
    comm.rotationalVelocity = 0.0;
    _motor_pub.publish(comm);

    ROS_INFO("Current time: %.3lf  expected position: (%.3lf,0.0,0.0) estimated position: (%.3lf,%.3lf,%.3lf)\n",
        cur_time-_time, 0.05*(cur_time-_time), _cur_loc.x, _cur_loc.y, _cur_loc.theta);
  }
}

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

double NormalizeRad(double rad) {
  rad = fmod(rad, 2*M_PI);
  if (rad < 0) {
    rad += 2*M_PI;
  }
  return rad;
}

} // namespace navigation
