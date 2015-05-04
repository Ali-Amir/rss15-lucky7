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
  _time_paint = CurTime();
  _cur_loc.x = 0.6;
  _cur_loc.y = 0.6;

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

  _stay_idle = 1;
  //PublishGUICSObstacles(ObstacleMap::ANGLE_DIVISIONS/4);

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
  ROS_INFO_STREAM_THROTTLE(10, "GOT POSITION UPDATE! TIME: "
      << CurTime()-_time << ". "
      << "Current location: (" << loc->x << " " << loc->y << " : "
      << ObstacleMap::RadToRotation(loc->theta) << ")");

  _cur_loc = *loc;

  moveRobotTo(nullptr);

  if (_testLocalization) {
    MotionMsg comm;
    comm.translationalVelocity = 0.05;
    comm.rotationalVelocity = 0.0;
    _motor_pub.publish(comm);

    ROS_INFO("Current time: %.3lf  expected position: (%.3lf,0.0,0.0) estimated position: (%.3lf,%.3lf,%.3lf)\n",
        CurTime()-_time, 0.05*(CurTime()-_time), _cur_loc.x, _cur_loc.y, _cur_loc.theta);
  }
}

bool Navigation::isLocationFree(
    LocFree::Request &req, LocFree::Response &res) {
  Grid::CellId cur_cell_id;
  assert(_world->GetCellId(Point_3(req.x, req.y,
                           ObstacleMap::RadToRotation(req.theta)),
                           &cur_cell_id));
  const Grid::Cell *cur_cell(_world->GetCell(cur_cell_id));
  res.result = cur_cell != nullptr;
  return true;
}

bool Navigation::UsePreviousCommand() {
  return CurTime() <= _time_until;
}

bool WithinReach(const RobotLocation &a, const RobotLocation &b) {
  double dtheta = NormalizeRad(a.theta-b.theta);
  if (dtheta > M_PI) {
    dtheta -= 2*M_PI;
  }
  if (sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)) < 2e-2 &&
      fabs(dtheta) < 3/180.0*M_PI) {
    return true;
  }
  return false;
}

void Navigation::moveRobotTo(const RobotLocation::ConstPtr &target) {
  double cur_time = ros::Time::now().toSec();
  ROS_DEBUG_THROTTLE(3, "Got a moveRobotTo command at navigation module.");
  bool isSame = false;
  if (target != nullptr) {
    double dx = _prev_target.x-target->x;
    double dy = _prev_target.y-target->y;
    double dt = _prev_target.theta-target->theta;
    if (fabs(dx) < 1e-2 && fabs(dy) < 1e-2 && fabs(dt) < 1e-2) {
      isSame = true;
    }
    _prev_target = *target;
  }

  if (target != nullptr) {
    ROS_INFO("COMMAND TO GO TO: %.3lf %.3lf %.3lf. isSame: %d", 
        target->x, target->y, target->theta, (int)isSame);
  }

  if (!UsePreviousCommand() && !_stay_idle) {
    MotionMsg stop_comm;
    stop_comm.translationalVelocity = 0.0;
    stop_comm.rotationalVelocity = 0.0;
    _motor_pub.publish(stop_comm);
  }

  int search_status = 1;
  vector<Grid::CellId> bfs_cells;

  if (target != nullptr && !isSame) {
    if (target->theta > 10.0) {
      _stay_idle = 1;
      MotionMsg stop_comm;
      stop_comm.translationalVelocity = 0.0;
      stop_comm.rotationalVelocity = 0.0;
      _motor_pub.publish(stop_comm);
    } else if (target->theta > -10.0) {
      _stay_idle = 0;
      bfs_cells = _world->ComputeInitCells(
          Point_3(target->x, target->y, ObstacleMap::RadToRotation(target->theta)),
                  &search_status);
    } else {
      _stay_idle = 0;
      bfs_cells = _world->ComputeInitCells(
          Point_2(target->x, target->y), &search_status);
    }
  }

  if (CurTime()-_time_paint > 200.0) {
    //PublishGUICSObstacles(ObstacleMap::RadToId(_cur_loc.theta));
    _time_paint = CurTime();
  }

  // search_status 0 means a new goal 
  if (!_stay_idle && (search_status == 0 || !UsePreviousCommand())) {
    MotionMsg stop_comm;
    stop_comm.translationalVelocity = 0.0;
    stop_comm.rotationalVelocity = 0.0;
    _motor_pub.publish(stop_comm);

    if (search_status == 0) {
      /*
      int cnt = 10;
      for (auto id : bfs_cells) {
        if (!cnt--) {
          break;
        }
        ROS_INFO("One of the start points: rotId %d", id.rotId);
      }
      */
      _world->_previous_start_ids = bfs_cells;
      _world->RunBfs(bfs_cells);
    }
    ROS_INFO_THROTTLE(3, "Search took: %.3lf sec. Search status=%d",
        ros::Time::now().toSec()-cur_time, search_status);

    ROS_INFO("Now calculating command velocities.");
    // Check if path exists.
    Grid::CellId cur_cell_id;
    assert(_world->GetCellId(
        Point_3(_cur_loc.x, _cur_loc.y,
                ObstacleMap::RadToRotation(_cur_loc.theta)),
        &cur_cell_id));
    const Grid::Cell *cur_cell(_world->GetCell(cur_cell_id));

    /*
    ROS_INFO("Current location: %.3lf %.3lf (rot:%.3lf) target: %.3lf %.3lf (rad:%.3lf).",
        _cur_loc.x, _cur_loc.y, ObstacleMap::RadToRotation(_cur_loc.theta),
        target->x, target->y, target->theta);
    */
    ROS_INFO("Cell that i got is: %d %d %d",
        cur_cell_id.r, cur_cell_id.c, cur_cell_id.rotId);

    if (cur_cell == nullptr || !cur_cell->HasPathToGoal()) {
      int curRotId = ObstacleMap::RadToId(_cur_loc.theta);
      auto free_cells = _world->getFreeCellsByRotId(curRotId);
      double best = -1.0;
      for (auto &cell : *free_cells) {
        ROS_INFO_THROTTLE(5, "good %d: %.3lf %.3lf",
            (int)cell->HasPathToGoal(), cell->xc, cell->yc);
        if (cell->HasPathToGoal()) {
          ROS_INFO_THROTTLE(5, "EVEN BETTER!!!");
          double cur = fabs(cell->xc-_cur_loc.x)+fabs(cell->yc-_cur_loc.y);

          if (best < 0 || cur < best) {
            best = cur;
            cur_cell = cell;
          }
        }
      }
      PublishGUICSObstacles(curRotId);
      ROS_INFO("Replaced with cell: %.3lf %.3lf %d",
          cur_cell->xc, cur_cell->yc, cur_cell->rotId);
      ROS_ASSERT(cur_cell != nullptr && cur_cell->HasPathToGoal());
    }
    /*
      double x = _cur_loc.x;
      double y = _cur_loc.y;
      double theta = _cur_loc.theta;
      double z = ObstacleMap::RadToRotation(theta);
      int id = ObstacleMap::RadToId(theta);
      int nxt = (id + 1) % ObstacleMap::ANGLE_DIVISIONS;
      int prev = (id - 1 + ObstacleMap::ANGLE_DIVISIONS) % ObstacleMap::ANGLE_DIVISIONS;
      ROS_INFO_STREAM("x: " << x << " y: " << y << " theta: " << theta);
      bool ok = 0;
      for (int i = -2; i <= 2; ++i) {
        int v = (id + ObstacleMap::ANGLE_DIVISIONS + i) % ObstacleMap::ANGLE_DIVISIONS;
        if (_obs_map->IsInsideObstacle(Point_2(x,y), v)) {
          ok = 1;
          PublishGUICSObstacles(v);
        }
      }
      ROS_ASSERT(ok);
    }
    */

    ROS_ASSERT(cur_cell != nullptr && cur_cell->HasPathToGoal());

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

    RobotLocation end;
    end.x = cur_cell->xc;
    end.y = cur_cell->yc;
    end.theta = ObstacleMap::IdToRad(cur_cell->rotId);
    if (WithinReach(end, _cur_loc)) {
      // TODO: publish message done!
    }

    // Calculate translational/rotational velocities
    GetSmoothPathVelocities(path);
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

  ROS_INFO_THROTTLE(10, "Commanding: %.3lf %.3lf time until next: %.3lf",
      _trans_velocity, _rot_velocity, _time_until-CurTime());

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
  int furthest = 0;
  for (; furthest < path.size(); ++furthest) {
    int i = furthest;
    double sqr_dist = CGAL::to_double(
        (path[i].x()-path[0].x())*(path[i].x()-path[0].x())+
        (path[i].y()-path[0].y())*(path[i].y()-path[0].y()));
    if (sqrt(sqr_dist) > 7e-2) {
      break; 
    }
  }

  for (;; --j) {
    double stax = CGAL::to_double(path[0].x());
    double stay = CGAL::to_double(path[0].y());
    double tarx = CGAL::to_double(path[j].x());
    double tary = CGAL::to_double(path[j].y());
    double d = sqrt((stax-tarx)*(stax-tarx)+(stay-tary)*(stay-tary));

    // Handle case when we need to rotate whithout translation.
    if (d < 2e-2) {//&& j < furthest) {
      double starot = CGAL::to_double(path[0].z());
      double tarrot = CGAL::to_double(path[j].z());
      double starad = ObstacleMap::RotationToRad(starot);
      // required dtheta in range (-pi,pi).
      double dtheta = NormalizeRad(ObstacleMap::RotationToRad(tarrot) -
                                   ObstacleMap::RotationToRad(starot));
      if (dtheta > M_PI) dtheta -= 2.0*M_PI;

      bool bad = 0;
      for (int step = 0; step < GRANULARITY; ++step) {
        Point_3 cur_point(stax, stay,
            ObstacleMap::RadToRotation(starad+step*dtheta/GRANULARITY));
        Grid::CellId cur_cell;
        if (!_world->GetCellId(cur_point, &cur_cell)) {
          //ROS_ASSERT(0);
          bad = 1;
        }
        if (_world->GetCell(cur_cell) == nullptr) {
          //ROS_ASSERT(0);
          bad = 1;
        }
      }

      if (!bad || j <= 2) {
        _trans_velocity = 0.0;
        _rot_velocity = sgn(dtheta)*MAX_ROT_VELOCITY;
        CapVelocities();
        _time_until = min(MAX_BLIND_TIME, fabs(dtheta*0.9/_rot_velocity))
                        + CurTime();
        if (fabs(starot-tarrot) < 1.0) {
          for (int i = 0; i < 5 && i < path.size(); ++i) {
            ROS_INFO("Point ahead: %.3lf %.3lf %.3lf",
                CGAL::to_double(path[i].x()),
                CGAL::to_double(path[i].y()),
                CGAL::to_double(path[i].z()));
          }
        }
        ROS_INFO("COMMANDING TO GO FROM %.3lf to %.3lf", starot, tarrot);
        return;
      }
      if (j <= 2) {
        ROS_ASSERT(0);
      }
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

    ROS_ASSERT(fabs(tar_rad-sta_rad) < M_PI+1e-8);

    // Split the path curve into GRANULARITY points and check each one of them
    // for passability.
    double curx = stax;
    double cury = stay;
    double cur_rad = sta_rad;
    bool ok = 1;
    double total_dist_so_far = 0.0;
    for (int step = 0; step < GRANULARITY; ++step) {
      Point_3 cur_point(curx, cury, ObstacleMap::RadToRotation(cur_rad));
      Grid::CellId cur_cell;
      if (!_world->GetCellId(cur_point, &cur_cell) && total_dist_so_far > BUFFER_SIZE) {
        ok = false;
        break;
      }
      if (_world->GetCell(cur_cell) == nullptr && total_dist_so_far > BUFFER_SIZE) {
        ok = false;
        break;
      }
      //auto cell = _world->GetCell(cur_cell);
      //ROS_INFO("CELL: %.3lf %.3lf %.3lf is reachable", cell->xc, cell->yc, ObstacleMap::IdToRad(cell->rotId));
      curx = curx + dir*mul*cos(cur_rad)/GRANULARITY;
      cury = cury + dir*mul*sin(cur_rad)/GRANULARITY;
      total_dist_so_far += fabs(mul/GRANULARITY);
      cur_rad = cur_rad + (tar_rad-sta_rad)/GRANULARITY;
    }

    // After we arrived at the required (x,y) coordinate, check if we can align as was required (we won't align
    // ourselves, but for the sake of consistency with our path we need to do this check).
    if (ok) {
      double rad1 = cur_rad;
      double rad2 = NormalizeRad(ObstacleMap::RotationToRad(CGAL::to_double(path[j].z())));
      double dtheta = NormalizeRad(rad2-rad1);
      if (dtheta > M_PI) dtheta -= 2.0*M_PI;

      for (int step = 0; step < GRANULARITY; ++step) {
        Point_3 cur_point(tarx, tary, ObstacleMap::RadToRotation(rad1 + step*dtheta/GRANULARITY));
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
      _rot_velocity = sgn(dtheta)*fabs(_trans_velocity)/radius;
      CapVelocities();
      if (is_straight) {
        _time_until = min(MAX_BLIND_TIME, d*0.9/fabs(_trans_velocity)) + CurTime();
      } else {
        _time_until = min(MAX_BLIND_TIME, fabs(dtheta*radius*0.9/_trans_velocity))
                        + CurTime();
      }
      ROS_INFO("COMMANDING TO GO FROM (%.2lf,%.2lf,%.3lf) to (%.2lf,%.2lf,%.3lf)", stax,stay,ObstacleMap::RadToRotation(sta_rad), tarx,tary,ObstacleMap::RadToRotation(tar_rad));
      return;
    }
  }
}

void Navigation::PublishGUICSObstacles(int lvl) {
  if (_prevLevel >= 0) {
    const auto &polys = _obs_map->_lvl_obstacles[_prevLevel];
    for (const shared_ptr<Polygon_2> &poly : polys) {
      GUIPolyMsg new_poly;
      new_poly.numVertices = poly->size();
      for (int i = 0; i < poly->size(); ++i) {
        new_poly.x.push_back(int(CGAL::to_double((*poly)[i].x())*1e3)/1e3);
        new_poly.y.push_back(int(CGAL::to_double((*poly)[i].y())*1e3)/1e3);
      }
      new_poly.c.r = 255;
      new_poly.c.g = 255;
      new_poly.c.b = 255;
      new_poly.closed = 1;
      _guipoly_pub.publish(new_poly);
    }
  }
  {
    const auto &polys = _obs_map->_lvl_obstacles[lvl];
    for (const shared_ptr<Polygon_2> &poly : polys) {
      GUIPolyMsg new_poly;
      new_poly.numVertices = poly->size();
      for (int i = 0; i < poly->size(); ++i) {
        new_poly.x.push_back(int(CGAL::to_double((*poly)[i].x())*1e3)/1e3);
        new_poly.y.push_back(int(CGAL::to_double((*poly)[i].y())*1e3)/1e3);
      }
      new_poly.c.r = 255;
      new_poly.c.g = 0;
      new_poly.c.b = 0;
      new_poly.closed = 1;
      _guipoly_pub.publish(new_poly);
    }
    _prevLevel = lvl;
  }
}

} // namespace navigation
