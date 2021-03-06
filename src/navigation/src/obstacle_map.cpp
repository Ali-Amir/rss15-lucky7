#include "obstacle_map.h"

#include <CGAL/algorithm.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Polyhedron_3.h>

#include <sstream>

using namespace std;

namespace cspace {

typedef cgal_kernel K;
typedef K::Segment_3 Segment_3;
typedef K::Point_2 Point_2;
typedef K::Point_3 Point_3;
typedef K::Ray_3 Ray;
typedef K::Iso_rectangle_2 Iso_rectangle_2;
typedef K::Vector_2 Vector_2;
typedef CGAL::Direction_3<K> Direction_3;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef CGAL::Polyhedron_3<K> Polyhedron_3;
typedef CGAL::AABB_polyhedron_triangle_primitive<K,Polyhedron_3> Primitive;
typedef CGAL::AABB_traits<K, Primitive> Traits;
typedef CGAL::AABB_tree<Traits> Tree;
typedef CGAL::Random_points_on_sphere_3<Point_3> Random_points_on_sphere_3;


ObstacleMap::ObstacleMap(const string &mapfile_location) :
  _point_gen(new Random_points_on_sphere_3(1.0)) {
  BuildMapFromFile(mapfile_location);
}

/***
 * Parses input file and builds cspace representation of the obstacle map.
 **/
void ObstacleMap::BuildMapFromFile(const string &mapfile_location) {
  // Parse and store map in member variables.
  ParseFromFile(mapfile_location);
  // Build cspace representation and store result in member variables.
  BuildCSpace();
}

/***
 * Parses the input file containing map information.
 **/
void ObstacleMap::ParseFromFile(const string &mapfile_location) {
  ROS_INFO("ObstacleMap: Parsing from file: %s", mapfile_location.c_str());

  FILE *fin;
  fin = fopen(mapfile_location.c_str(), "r");
  ParsePoint(fin, &_robot_start);
  ParsePoint(fin, &_robot_goal);
  ParseRect(fin, &_world_rect);

  int numObstacles;
  ROS_ASSERT(fscanf(fin, "%d", &numObstacles)==1);

  _raw_obstacles.clear();
  for (int obstacleNumber = 0; obstacleNumber < numObstacles;
      ++obstacleNumber) {
    Polygon_2 poly;
    if (!ParseObstacle(fin, &poly)) {
      break;
    }
    _raw_obstacles.push_back(poly);
  }

  fclose(fin);

  {
    Polygon_2 upper_wall;
    upper_wall.insert(upper_wall.vertices_end(),
                      Point_2(_world_rect.xmin(), _world_rect.ymax()));
    upper_wall.insert(upper_wall.vertices_end(),
                      Point_2(_world_rect.xmax(), _world_rect.ymax()));
    _raw_obstacles.push_back(upper_wall);
  }
  {
    Polygon_2 lower_wall;
    lower_wall.insert(lower_wall.vertices_end(),
                      Point_2(_world_rect.xmin(), _world_rect.ymin()));
    lower_wall.insert(lower_wall.vertices_end(),
                      Point_2(_world_rect.xmax(), _world_rect.ymin()));
    _raw_obstacles.push_back(lower_wall);
  }
  {
    Polygon_2 left_wall;
    left_wall.insert(left_wall.vertices_end(),
                      Point_2(_world_rect.xmin(), _world_rect.ymin()));
    left_wall.insert(left_wall.vertices_end(),
                      Point_2(_world_rect.xmin(), _world_rect.ymax()));
    _raw_obstacles.push_back(left_wall);
  }
  {
    Polygon_2 right_wall;
    right_wall.insert(right_wall.vertices_end(),
                      Point_2(_world_rect.xmax(), _world_rect.ymin()));
    right_wall.insert(right_wall.vertices_end(),
                      Point_2(_world_rect.xmax(), _world_rect.ymax()));
    _raw_obstacles.push_back(right_wall);
  }

  ROS_INFO("ObstacleMap: Parsing done!");
}

/***
 * Parses a 2d point from input stream.
 **/
void ObstacleMap::ParsePoint(FILE *fin, Point_2 *point) {
  double x, y;
  ROS_ASSERT(fscanf(fin, "%lf%lf", &x, &y)==2);
  *point = Point_2(x, y);
}

/***
 * Parses world boundary rectangle from input stream.
 **/
void ObstacleMap::ParseRect(FILE *fin, Iso_rectangle_2 *rect) {
  double x, y, xx, yy;
  ROS_ASSERT(fscanf(fin, "%lf%lf%lf%lf", &x, &y, &xx, &yy)==4);
  Point_2 corner(x, y), cornerx(x+xx, y+yy);
  *rect = Iso_rectangle_2(corner, cornerx);
  double a = CGAL::to_double(rect->xmin()),
         b = CGAL::to_double(rect->ymin()),
         c = CGAL::to_double(rect->xmax()),
         d = CGAL::to_double(rect->ymax());
  ROS_INFO("World bounds: %.3lf %.3lf -- %.3lf %.3lf\n", a, b, c, d);
}

/***
 * Parses raw obstacles from input stream.
 **/
bool ObstacleMap::ParseObstacle(FILE *fin, Polygon_2 *poly) {
  *poly = Polygon_2();

  int n;
  ROS_ASSERT(fscanf(fin, "%d", &n)==1);
  for (int i = 0; i < n; ++i) {
    double x, y;
    ROS_ASSERT(fscanf(fin, "%lf%lf", &x, &y)==2);
    poly->insert(poly->vertices_end(), Point_2(x, y));
  }

  return true;
}

bool ObstacleMap::IsInsideObstacle(const Point_2 &point) {
  for (const Polygon_2 &poly : _raw_obstacles) {
    if (poly.bounded_side(point) == CGAL::ON_BOUNDED_SIDE) {
      return true;
    }
  }
  return false;
}

/***
 * Builds the CSpace representation given raw obstacles in 2D.
 **/
void ObstacleMap::BuildCSpace() {
  double start_time_sec = ros::Time::now().toSec();

  ROS_INFO("ObstacleMap: Building CSpace");
  _obstacles_tree.clear();
  ROS_INFO("ObstacleMap: Cleared obstacle tree");
  for (Polygon_2 raw_poly : _raw_obstacles) {
    ROS_INFO_ONCE("ObstacleMap: Adding a raw obstacle");

    vector<Point_3> points;
    for (int rotInd = 0; rotInd < ANGLE_DIVISIONS; ++rotInd) {
      shared_ptr<Polygon_2> csobstacle2d(new Polygon_2()); 
      GetCSpaceObstacle2d(raw_poly, rotInd, csobstacle2d.get());
      _lvl_obstacles[rotInd].push_back(csobstacle2d);
      for (int i = 0; i < csobstacle2d->size(); ++i) {
        points.push_back(Point_3((*csobstacle2d)[i].x(),
                                 (*csobstacle2d)[i].y(),
                                 IdToRotation(rotInd)));
      }
    }

    ROS_INFO_ONCE("ObstacleMap: Added the points of a polyhedron."
                  "Now constructing!");
    shared_ptr<Polyhedron_3> polyhedron(new Polyhedron_3());
    CGAL::convex_hull_3(points.begin(), points.end(), *polyhedron);
    _obstacles_tree.insert(polyhedron->facets_begin(), polyhedron->facets_end());
    _obs_polyhedra.push_back(polyhedron);
  }
  ROS_INFO("ObstacleMap: Adding obstacles took: %.3lf sec",
      ros::Time::now().toSec()-start_time_sec);
  // Make distance queries faster by doing precomputation.
  ROS_INFO("ObstacleMap: Accelerating distance queries!");
  _obstacles_tree.accelerate_distance_queries();
  ROS_INFO("ObstacleMap: Done Building CSpace!");
}

double ObstacleMap::GetWorldWidth() {
  return CGAL::to_double(_world_rect.xmax()-_world_rect.xmin());
}

double ObstacleMap::GetWorldHeight() {
  return CGAL::to_double(_world_rect.ymax()-_world_rect.ymin());
}

double ObstacleMap::GetWorldCenterX() {
  return CGAL::to_double(_world_rect.xmin()+_world_rect.xmax())*0.5;
}

double ObstacleMap::GetWorldCenterY() {
  return CGAL::to_double(_world_rect.ymin()+_world_rect.ymax())*0.5;
}

bool ObstacleMap::WorldContains(const Point_2& point) {
  return _world_rect.xmin() < point.x()+1e-9 &&
         point.x() < _world_rect.xmax()+1e-9 &&
         _world_rect.ymin() < point.y()+1e-9 &&
         point.y() < _world_rect.ymax()+1e-9;
}

bool ObstacleMap::WorldContains(const Point_3& point) {
  return _world_rect.xmin() < point.x()+1e-9 &&
         point.x() < _world_rect.xmax()+1e-9 &&
         _world_rect.ymin() < point.y()+1e-9 &&
         point.y() < _world_rect.ymax()+1e-9;
}

int ObstacleMap::RadToId(double rad) {
  return RotationToId(RadToRotation(rad));
}

int ObstacleMap::RotationToId(double rotation) {
  assert(rotation > -1e-9);
  assert(rotation < 360.0+1e-9);
  return min(max(0, (int)round(ANGLE_DIVISIONS*rotation/360.0)),
             ANGLE_DIVISIONS-1);
}

double ObstacleMap::IdToRotation(int id) {
  return (360.0*id)/ANGLE_DIVISIONS;
}

double ObstacleMap::IdToRad(int id) {
  return (2.0*M_PI*id)/ANGLE_DIVISIONS;
}

double ObstacleMap::RadToRotation(double rad) {
  rad = fmod(rad, 2*M_PI);
  if (rad < 0) {
    rad += 2*M_PI;
  }
  return rad/M_PI*180.0;
}

double ObstacleMap::RotationToRad(double rotation) {
  return rotation/180.0*M_PI;
}

double ObstacleMap::DistanceToClosestObstacle(const Point_3 &point) {
  return sqrt(CGAL::to_double(_obstacles_tree.squared_distance(point)));
}

bool ObstacleMap::IsInsideObstacle(const Point_3 &point) {
  Ray ray_query(point, Direction_3((*_point_gen)->x(), (*_point_gen)->y(),
                                   (*_point_gen)->z()));
  (*_point_gen)++;
  return !(_obstacles_tree.number_of_intersected_primitives(ray_query)&1);
}

bool ObstacleMap::IsInsideObstacle(const Point_2 &point, const int rotInd) {
  for (auto poly_ptr : _lvl_obstacles[rotInd]) {
    if (poly_ptr->bounded_side(point) == CGAL::ON_BOUNDED_SIDE) {
      //ROS_INFO_THROTTLE(5, "Point intersects polygon!");
      return true;
    }
  }
  return false;
}

bool ObstacleMap::IsObstacleFree(const Point_3 &point, const double radius) {
  if (IsInsideObstacle(point)) {
    return false;
  }
  if (DistanceToClosestObstacle(point) < radius) {
    return false;
  }
  return true;
}

} // namespace cspace 
