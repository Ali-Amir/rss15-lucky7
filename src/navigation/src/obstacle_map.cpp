#include "obstacle_map.h"

#include "cspace_tools.h"

#include <CGAL/algorithm.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Polyhedron_3.h>

#include <cassert>
#include <sstream>

using namespace std;

namespace cspace {

typedef CGAL::Simple_cartesian<double> K;
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
  ifstream mapstream;
  mapstream.open(mapfile_location, ifstream::in);
  ParsePoint(mapstream, &_robot_start);
  ParsePoint(mapstream, &_robot_goal);
  ParseRect(mapstream, &_world_rect);

  _raw_obstacles.clear();
  for (int obstacleNumber = 0;; ++obstacleNumber) {
    Polygon_2 poly;
    if (!ParseObstacle(mapstream, &poly)) {
      break;
    }
    _raw_obstacles.push_back(poly);
  }
}

/***
 * Parses a 2d point from input stream.
 **/
void ObstacleMap::ParsePoint(ifstream &stream, Point_2 *point) {
  char buff[1024];
  stream.getline(buff, 1024);
  double x, y;
  stringstream line;
  line << string(buff);
  line >> x >> y;
  *point = Point_2(x, y);
}

/***
 * Parses world boundary rectangle from input stream.
 **/
void ObstacleMap::ParseRect(ifstream &stream, Iso_rectangle_2 *rect) {
  Point_2 corner, size;
  ParsePoint(stream, &corner);
  ParsePoint(stream, &size);
  *rect = Iso_rectangle_2(corner, corner+Vector_2(size.x(), size.y()));
}

/***
 * Parses raw obstacles from input stream.
 **/
bool ObstacleMap::ParseObstacle(ifstream &stream, Polygon_2 *poly) {
  if (stream.eof()) {
    return false;
  }

  *poly = Polygon_2();

  char buff[1024];
  stream.getline(buff, 1024);
  stringstream line;
  line << string(buff);
  double x, y;
  while (line >> x >> y) {
    poly->insert(poly->vertices_end(), Point_2(x, y));
  }

  assert(!poly->is_empty());

  return true;
}

/***
 * Builds the CSpace representation given raw obstacles in 2D.
 **/
void ObstacleMap::BuildCSpace() {
  _obstacles_tree.clear();
  for (Polygon_2 raw_poly : _raw_obstacles) {
    vector<Point_3> points;
    for (int rotInd = 0; rotInd < ANGLE_DIVISIONS; ++rotInd) {
      Polygon_2 csobstacle2d; 
      GetCSpaceObstacle2d(raw_poly, rotInd, &csobstacle2d);
      for (int i = 0; i < csobstacle2d.size(); ++i) {
        points.push_back(Point_3(csobstacle2d[i].x(),
                         csobstacle2d[i].y(),
                         IdToRotation(rotInd)*360.0/ANGLE_DIVISIONS));
      }
    }

    Polyhedron_3 polyhedron;
    CGAL::convex_hull_3(points.begin(), points.end(), polyhedron);
    _obstacles_tree.insert(polyhedron.facets_begin(), polyhedron.facets_end());
  }
  // Make distance queries faster by doing precomputation.
  _obstacles_tree.accelerate_distance_queries();
}

double ObstacleMap::GetWorldWidth() {
  return _world_rect.xmax()-_world_rect.xmin();
}

double ObstacleMap::GetWorldHeight() {
  return _world_rect.ymax()-_world_rect.ymin();
}

double ObstacleMap::GetWorldCenterX() {
  return (_world_rect.xmin()+_world_rect.xmax())*0.5;
}

double ObstacleMap::GetWorldCenterY() {
  return (_world_rect.ymin()+_world_rect.ymax())*0.5;
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

int ObstacleMap::RotationToId(double rotation) {
  return int(ANGLE_DIVISIONS/360.0*rotation+1e-9);
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
  return rad/2/M_PI*360.0;
}

double ObstacleMap::DistanceToClosestObstacle(const Point_3 &point) {
  return sqrt(_obstacles_tree.squared_distance(point));
}

bool ObstacleMap::IsInsideObstacle(const Point_3 &point) {
  Ray ray_query(point, Direction_3((*_point_gen)->x(), (*_point_gen)->y(),
                                   (*_point_gen)->z()));
  (*_point_gen)++;
  return !(_obstacles_tree.number_of_intersected_primitives(ray_query)&1);
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
