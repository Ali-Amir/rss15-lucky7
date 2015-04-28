#include "wall_map.h"

#include <CGAL/algorithm.h>

#include <sstream>

using namespace std;

namespace localization {

typedef cgal_kernel K;
typedef K::Segment_3 Segment_3;
typedef K::Point_2 Point_2;
typedef K::Point_3 Point_3;
typedef K::Ray_2 Ray_2;
typedef K::Iso_rectangle_2 Iso_rectangle_2;
typedef K::Vector_2 Vector_2;
typedef CGAL::Direction_3<K> Direction_3;
typedef CGAL::Polygon_2<K> Polygon_2;


WallMap::WallMap(const string &mapfile_location) {
  BuildMapFromFile(mapfile_location);
}

/***
 * Parses input file and builds cspace representation of the obstacle map.
 **/
void WallMap::BuildMapFromFile(const string &mapfile_location) {
  // Parse and store map in member variables.
  ParseFromFile(mapfile_location);
  // Build cspace representation and store result in member variables.
  BuildTriangles();
}

/***
 * Parses the input file containing map information.
 **/
void WallMap::ParseFromFile(const string &mapfile_location) {
  ROS_INFO("WallMap: Parsing from file: %s", mapfile_location.c_str());

  ifstream mapstream;
  mapstream.open(mapfile_location, ifstream::in);
  Point_2 tmp_point;
  ParsePoint(mapstream, &tmp_point);
  ParsePoint(mapstream, &tmp_point);
  Iso_rectangle_2 rect;
  ParseRect(mapstream, &rect);

  _raw_obstacles.clear();
  for (int obstacleNumber = 0;; ++obstacleNumber) {
    Polygon_2 poly;
    if (!ParseObstacle(mapstream, &poly)) {
      break;
    }
    _raw_obstacles.push_back(poly);
  }
  {
    Polygon_2 upper_wall;
    upper_wall.insert(upper_wall.vertices_end(),
                      Point_2(rect.xmin(), rect.ymax()));
    upper_wall.insert(upper_wall.vertices_end(),
                      Point_2(rect.xmax(), rect.ymax()));
    _raw_obstacles.push_back(upper_wall);
  }
  {
    Polygon_2 lower_wall;
    lower_wall.insert(lower_wall.vertices_end(),
                      Point_2(rect.xmin(), rect.ymin()));
    lower_wall.insert(lower_wall.vertices_end(),
                      Point_2(rect.xmax(), rect.ymin()));
    _raw_obstacles.push_back(lower_wall);
  }
  {
    Polygon_2 left_wall;
    left_wall.insert(left_wall.vertices_end(),
                      Point_2(rect.xmin(), rect.ymin()));
    left_wall.insert(left_wall.vertices_end(),
                      Point_2(rect.xmin(), rect.ymax()));
    _raw_obstacles.push_back(left_wall);
  }
  {
    Polygon_2 right_wall;
    right_wall.insert(right_wall.vertices_end(),
                      Point_2(rect.xmax(), rect.ymin()));
    right_wall.insert(right_wall.vertices_end(),
                      Point_2(rect.xmax(), rect.ymax()));
    _raw_obstacles.push_back(right_wall);
  }

  ROS_INFO("WallMap: Parsing done!");
}

/***
 * Parses a 2d point from input stream.
 **/
void WallMap::ParsePoint(ifstream &stream, Point_2 *point) {
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
void WallMap::ParseRect(ifstream &stream, Iso_rectangle_2 *rect) {
  char buff[1024];
  stream.getline(buff, 1024);
  stringstream line;
  line << string(buff);
  double x, y, size_x, size_y;
  line >> x >> y >> size_x >> size_y;
  Point_2 corner(x, y), size(size_x, size_y);
  *rect = Iso_rectangle_2(corner, corner+Vector_2(size.x(), size.y()));
}

/***
 * Parses raw obstacles from input stream.
 **/
bool WallMap::ParseObstacle(ifstream &stream, Polygon_2 *poly) {
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

  if (poly->is_empty()) {
    return false;
  }

  return true;
}

void WallMap::BuildTriangles() {
  double start_time_sec = ros::Time::now().toSec();

  ROS_INFO("WallMap: Building Triangles");
  _triangles.clear();
  for (Polygon_2 raw_poly : _raw_obstacles) {
    ROS_INFO_ONCE("WallMap: Adding a triangulation of an obstacle");

    /*
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

    ROS_INFO_ONCE("WallMap: Added the points of a polyhedron."
                  "Now constructing!");
    shared_ptr<Polyhedron_3> polyhedron(new Polyhedron_3());
    CGAL::convex_hull_3(points.begin(), points.end(), *polyhedron);
    _obstacles_tree.insert(polyhedron->facets_begin(), polyhedron->facets_end());
    _obs_polyhedra.push_back(polyhedron);
    */
  }
  ROS_INFO("WallMap: Adding %d triangles took: %.3lf sec.",
      int(_triangles.size()), ros::Time::now().toSec()-start_time_sec);
  ROS_INFO("WallMap: Done Building Triangulation!");
}

double WallMap::DistanceToWall(const Ray_2 &point) {
  // TODO
  return 0.0;
}

} // namespace localization 
