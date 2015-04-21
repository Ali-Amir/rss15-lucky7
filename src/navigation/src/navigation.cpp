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

typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_2 Point_2;
typedef K::Point_3 Point_3;

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
  assert(n.getParam("~/mapFileName", mapfile_location));
  ROS_INFO("Got mapFileName: %s", mapfile_location.c_str());
  _obs_map = make_shared<ObstacleMap>(mapfile_location);
  ROS_INFO("Initialized obstacle map.");

  // Initialize the grid graph.
  _world.reset(new Grid(_obs_map));
  ROS_INFO("Initialized world grid.");

  // TODO: Add localization subscriber.
}

void Navigation::updateRobotLocation(const RobotLocation::ConstPtr &loc) {
  _cur_loc = *loc;
}

void Navigation::moveRobotTo(const RobotLocation::ConstPtr &target) {
  ROS_INFO("Got a moveRobotTo command at navigation module.");

  _world->ComputePathsToGoal(Point_2(target->x, target->y));
  Grid::CellId cur_cell_id;
  assert(_world->GetCellId(
        Point_3(_cur_loc.x, _cur_loc.y,
                ObstacleMap::RadToRotation(_cur_loc.theta)),
        &cur_cell_id));
  shared_ptr<const Grid::Cell> cur_cell(_world->GetCell(cur_cell_id));
  assert(cur_cell->HasPathToGoal());

  vector<Point_3> path;
  for (; cur_cell->to_goal_next != nullptr;) {
    path.push_back(Point_3(cur_cell->xc, cur_cell->yc,
                           ObstacleMap::IdToRotation(cur_cell->rotId)));
  }
  // TODO: Smoothen the path, etc.

  GUIPolyMsg path_poly;
  path_poly.numVertices = path.size();
  for (const Point_3 &p : path) {
    path_poly.x.push_back(p.x());
    path_poly.y.push_back(p.y());
  }
  path_poly.c.r = 255;
  path_poly.c.g = 255;
  path_poly.c.b = 255;
  _guipoly_pub.publish(path_poly);
}

} // namespace navigation
