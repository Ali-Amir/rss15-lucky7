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

double CurTime() {
  return ros::Time::now().toSec();
}

double sgn(double x) {
  return x > 0 ? 1.0 : x < 0.0 ? -1.0 : 0.0;
}

double NormalizeRad(double rad) {
  rad = fmod(rad, 2*M_PI);
  if (rad < 0) {
    rad += 2*M_PI;
  }
  return rad;
}

Navigation::Navigation() {
  ROS_INFO("Initializing navigation module");

  _time = CurTime();

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
  // TEST CASE 03
  {
    string res;
    if (n.getParam("/nav/testTrickyNavigation", res)) {
      if (res == "yes") {
        boost::shared_ptr<RobotLocation> loc(new RobotLocation());
        loc->x = CGAL::to_double(_obs_map->_robot_goal.x());
        loc->y = CGAL::to_double(_obs_map->_robot_goal.y());
        loc->theta = ObstacleMap::RadToRotation(-M_PI);
        loc->x -= 0.10;
        _cur_loc = *loc;
        _cur_loc.theta = ObstacleMap::RadToRotation(0.0);
        moveRobotTo(loc);
      }
    }
  }
  // TODO: remove later, after visualization is not needed
  PublishGUICSObstacles();
}

void Navigation::TestWheelVelocities() {
  double start_time = CurTime();
  double cur_time;
  while ((cur_time = CurTime()) - start_time < 10.0) {
    MotionMsg comm;
    comm.translationalVelocity = 0.0;
    comm.rotationalVelocity = M_PI/10.0;
    _motor_pub.publish(comm);
  }
  ROS_INFO("Time taken: %.3lf\n", cur_time-start_time);
  MotionMsg comm;
  comm.translationalVelocity = 0.0;
  comm.rotationalVelocity = 0.0;
  _motor_pub.publish(comm);
}

void Navigation::updateRobotLocation(const RobotLocation::ConstPtr &loc) {
  ROS_INFO("GOT POSITION UPDATE! TIME: %.3lf\n", CurTime()-_time);

  _cur_loc = *loc;

  if (_testLocalization) {
    MotionMsg comm;
    comm.translationalVelocity = 0.05;
    comm.rotationalVelocity = 0.0;
    _motor_pub.publish(comm);

    ROS_INFO("Current time: %.3lf  expected position: (%.3lf,0.0,0.0) estimated position: (%.3lf,%.3lf,%.3lf)\n",
        CurTime()-_time, 0.05*(CurTime()-_time), _cur_loc.x, _cur_loc.y, _cur_loc.theta);
  }
}

void Navigation::PublishGUICSObstacles() {
  const auto &polys = _obs_map->_lvl_obstacles[ObstacleMap::ANGLE_DIVISIONS/4];
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
}

bool Navigation::UsePreviousCommand() {
  return CurTime() >= _time_until;
}

void Navigation::moveRobotTo(const RobotLocation::ConstPtr &target) {
  double cur_time = ros::Time::now().toSec();
  ROS_INFO("Got a moveRobotTo command at navigation module.");

  int search_status;
  if (target->theta > -10.0) {
    _world->ComputePathsToGoal(Point_3(target->x, target->y, target->theta), &search_status);
  } else {
    _world->ComputePathsToGoal(Point_2(target->x, target->y), &search_status);
  }

  ROS_INFO("Computed all paths. Now calculating command velocities.");
  ROS_INFO("Search took: %.3lf sec.", ros::Time::now().toSec()-cur_time);

  // search_status 0 means a new goal 
  if (search_status == 0 || !UsePreviousCommand()) {
    // Check if path exists.
    Grid::CellId cur_cell_id;
    assert(_world->GetCellId(
          Point_3(_cur_loc.x, _cur_loc.y,
                  ObstacleMap::RadToRotation(_cur_loc.theta)),
          &cur_cell_id));
    const Grid::Cell *cur_cell(_world->GetCell(cur_cell_id));

    ROS_ASSERT(cur_cell->HasPathToGoal());

    ROS_INFO("Current location: %.3lf %.3lf target: %.3lf %.3lf. Path length: %.3lf",
        _cur_loc.x, _cur_loc.y, target->x, target->y, cur_cell->min_dist_to_goal);

    // Get the path.
    vector<Point_3> path;
    path.push_back(Point_3(_cur_loc.x, _cur_loc.y, ObstacleMap::RadToRotation(_cur_loc.theta)));
    for (; cur_cell->to_goal_next != nullptr; cur_cell = cur_cell->to_goal_next) {
      //ROS_INFO("Point: %.3lf %.3lf angle: %.3lf", cur_cell->xc, cur_cell->yc, ObstacleMap::IdToRotation(cur_cell->rotId));
      path.push_back(Point_3(cur_cell->xc, cur_cell->yc,
                             ObstacleMap::IdToRotation(cur_cell->rotId)));
    }
    path.push_back(Point_3(cur_cell->xc, cur_cell->yc,
                           ObstacleMap::IdToRotation(cur_cell->rotId)));

    // Calculate translational/rotational velocities
    GetSmoothPathVelocities(path);
    CapVelocities();
    //ROS_INFO("Publishing the path of size: %d", int(smooth_path.size()));

    /*
    GUIPolyMsg path_poly;
    path_poly.numVertices = smooth_path.size();
    for (const Point_3 &p : smooth_path) {
      ROS_INFO("Point: %.3lf %.3lf angle: %.3lf", CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z()));
      path_poly.x.push_back(CGAL::to_double(p.x()));
      path_poly.y.push_back(CGAL::to_double(p.y()));
    }
    path_poly.c.r = 255;
    path_poly.c.g = 0;
    path_poly.c.b = 0;
    _guipoly_pub.publish(path_poly);
    */

    MotionMsg mot_msg;
    mot_msg.translationalVelocity = _trans_velocity;
    mot_msg.rotationalVelocity = _rot_velocity;
    _motor_pub.publish(mot_msg);
  }

}

void Navigation::CapVelocities() {
  double norm = 1.0;
  norm = max(norm, fabs(_trans_velocity)/MAX_TRANS_VELOCITY);
  norm = max(norm, fabs(_rot_velocity)/MAX_ROT_VELOCITY);
  _trans_velocity /= norm;
  _rot_velocity /= norm;
}

void Navigation::GetSmoothPathVelocities(const vector<Point_3> &path) {
  int j = path.size()-1;
  vector<Point_3> smooth;
  for (;; --j) {
    double stax = CGAL::to_double(path[0].x());
    double stay = CGAL::to_double(path[0].y());
    double tarx = CGAL::to_double(path[j].x());
    double tary = CGAL::to_double(path[j].y());
    double d = sqrt((stax-tarx)*(stax-tarx)+(stay-tary)*(stay-tary));

    // Handle case when we need to rotate whithout translation.
    if (d < 2e-2 && j <= 2) {
      double starot = CGAL::to_double(path[0].z());
      double tarrot = CGAL::to_double(path[j].z());
      for (int step = 0; step < GRANULARITY; ++step) {
        Point_3 cur_point(stax, stay, starot+step*(tarrot-starot)/GRANULARITY);
        Grid::CellId cur_cell;
        if (!_world->GetCellId(cur_point, &cur_cell)) {
          ROS_ASSERT(0);
        }
        if (_world->GetCell(cur_cell) == nullptr) {
          ROS_ASSERT(0);
        }
      }

      // required dtheta in range (-pi,pi).
      double dtheta = NormalizeRad(ObstacleMap::RotationToRad(tarrot) -
                                   ObstacleMap::RotationToRad(starot));
      if (dtheta > M_PI) dtheta -= 2.0*M_PI;

      _trans_velocity = 0.0;
      _rot_velocity = sgn(dtheta)*MAX_ROT_VELOCITY;
      _time_until = min(MIN_BLIND_TIME, CurTime() + fabs(dtheta/_rot_velocity));
      return;
    }

    // Calculate start heading, as well as the target heading, such that the
    // path is part of a circle.
    double heading_rad = atan2(tary-stay, tarx-stax);
    double sta_rad = NormalizeRad(ObstacleMap::RotationToRad(
          CGAL::to_double(path[0].z())));
    double tar_rad = NormalizeRad(heading_rad-(sta_rad-heading_rad));
    if (sta_rad < tar_rad && tar_rad-sta_rad > M_PI) {
      sta_rad += 2.0*M_PI;
    }
    if (sta_rad > tar_rad && sta_rad-tar_rad > M_PI) {
      tar_rad += 2.0*M_PI;
    }

    // Decide on the direction, such that we do not turn more than 90 degrees in
    // any direction.
    double dir;
    if (cos(heading_rad-sta_rad) < 0.0) {
      dir = -1.0;
    } else {
      dir = 1.0;
    }

    // Check if the motion is along the straight line to avoid division by zero.
    bool is_straight = fabs(sin(heading_rad-sta_rad)) < 1e-6;
    double mul;
    if (fabs(sin(heading_rad-sta_rad)) < 1e-6) {
      mul = d;
    } else {
      double radius = d / 2.0 / abs(sin(heading_rad-sta_rad));
      mul = fabs(sta_rad-tar_rad)*radius;
    }
    double radius = is_straight ? 1e18 : (d / 2.0 / abs(sin(heading_rad-sta_rad)));

    // Split the path curve into GRANULARITY points and check each one of them
    // for passability.
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
      //auto cell = _world->GetCell(cur_cell);
      //ROS_INFO("CELL: %.3lf %.3lf %.3lf is reachable", cell->xc, cell->yc, ObstacleMap::IdToRad(cell->rotId));
      curx = curx + dir*mul*cos(cur_rad)/GRANULARITY;
      cury = cury + dir*mul*sin(cur_rad)/GRANULARITY;
      cur_rad = cur_rad + (tar_rad-sta_rad)/GRANULARITY;
    }

    // After we arrived at the required (x,y) coordinate, check if we can align as was required (we won't align
    // ourselves, but for the sake of consistency with our path we need to do this check).
    if (ok) {
      double rad1 = cur_rad;
      double rad2 = NormalizeRad(ObstacleMap::RotationToRad(CGAL::to_double(path[j].z())));
      for (int step = 0; step < GRANULARITY; ++step) {
        Point_3 cur_point(tarx, tary, ObstacleMap::RadToRotation(rad1 + step*(rad2-rad1)/GRANULARITY));
        Grid::CellId cur_cell;
        if (!_world->GetCellId(cur_point, &cur_cell)) {
          ok = false;
          break;
        }
        if (_world->GetCell(cur_cell) == nullptr) {
          ok = false;
          break;
        }
      }
    }

    // If a circular path is possible, just use its parameters for the command.
    if (ok) {
      double dtheta = NormalizeRad(tar_rad-sta_rad);
      if (dtheta > M_PI) dtheta -= 2.0*M_PI;

      _trans_velocity = dir*MAX_TRANS_VELOCITY;
      _rot_velocity = sgn(dtheta)*_trans_velocity/radius;
      if (is_straight) {
        _time_until = d / fabs(_trans_velocity);
      } else {
        _time_until = CurTime() + fabs(dtheta*radius/_trans_velocity);
      }
      return;
    }
  }
}

} // namespace navigation
