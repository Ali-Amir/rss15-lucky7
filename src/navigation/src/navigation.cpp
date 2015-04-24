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
}

void Navigation::updateRobotLocation(const RobotLocation::ConstPtr &loc) {
  _cur_loc = *loc;
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
  for (; cur_cell->to_goal_next != nullptr; cur_cell = cur_cell->to_goal_next) {
    //ROS_INFO("Point: %.3lf %.3lf angle: %.3lf", cur_cell->xc, cur_cell->yc, ObstacleMap::IdToRotation(cur_cell->rotId));
    path.push_back(Point_3(cur_cell->xc, cur_cell->yc,
                           ObstacleMap::IdToRotation(cur_cell->rotId)));
  }
  path.push_back(Point_3(cur_cell->xc, cur_cell->yc,
                         ObstacleMap::IdToRotation(cur_cell->rotId)));
  // TODO: Smoothen the path, etc.
  ROS_INFO("Publishing the path of size: %d", int(path.size()));

  GUIPolyMsg path_poly;
  path_poly.numVertices = path.size();
  for (const Point_3 &p : path) {
    path_poly.x.push_back(CGAL::to_double(p.x()));
    path_poly.y.push_back(CGAL::to_double(p.y()));
  }
  path_poly.c.r = 255;
  path_poly.c.g = 0;
  path_poly.c.b = 0;
  _guipoly_pub.publish(path_poly);
}

} // namespace navigation
