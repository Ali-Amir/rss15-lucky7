#include "wall_map.h"

#include <CGAL/algorithm.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/intersections.h>
#include <CGAL/squared_distance_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>

#include <sstream>

using namespace std;

namespace localization {

typedef cgal_kernel K;
typedef CGAL::Point_2<K> Point_2;
typedef CGAL::Ray_2<K> Ray_2;
typedef CGAL::Segment_2<K> Segment_2;
typedef K::Iso_rectangle_2 Iso_rectangle_2;
typedef K::Vector_2 Vector_2;
typedef CGAL::Direction_3<K> Direction_3;
typedef CGAL::Polygon_2<K> Polygon_2;

typedef CGAL::Triangulation_vertex_base_2<K>                     Vb;
typedef CGAL::Constrained_triangulation_face_base_2<K>           Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>              TDS;
typedef CGAL::Exact_predicates_tag                               Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag> CDT;


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
  ParsePoint(mapstream, &_robot_start);
  Point_2 tmp_point;
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
                      Point_2(rect.xmin(), rect.ymax()+0.1));
    upper_wall.insert(upper_wall.vertices_end(),
                      Point_2(rect.xmax(), rect.ymax()+0.1));
    upper_wall.insert(upper_wall.vertices_end(),
                      Point_2(rect.xmax(), rect.ymax()));
    upper_wall.insert(upper_wall.vertices_end(),
                      Point_2(rect.xmin(), rect.ymax()));
    _raw_obstacles.push_back(upper_wall);
  }
  {
    Polygon_2 lower_wall;
    lower_wall.insert(lower_wall.vertices_end(),
                      Point_2(rect.xmin(), rect.ymin()));
    lower_wall.insert(lower_wall.vertices_end(),
                      Point_2(rect.xmax(), rect.ymin()));
    lower_wall.insert(lower_wall.vertices_end(),
                      Point_2(rect.xmax(), rect.ymin()-0.1));
    lower_wall.insert(lower_wall.vertices_end(),
                      Point_2(rect.xmin(), rect.ymin()-0.1));
    _raw_obstacles.push_back(lower_wall);
  }
  {
    Polygon_2 left_wall;
    left_wall.insert(left_wall.vertices_end(),
                      Point_2(rect.xmin()-0.1, rect.ymin()));
    left_wall.insert(left_wall.vertices_end(),
                      Point_2(rect.xmin(), rect.ymin()));
    left_wall.insert(left_wall.vertices_end(),
                      Point_2(rect.xmin(), rect.ymax()));
    left_wall.insert(left_wall.vertices_end(),
                      Point_2(rect.xmin()-0.1, rect.ymax()));
    _raw_obstacles.push_back(left_wall);
  }
  {
    Polygon_2 right_wall;
    right_wall.insert(right_wall.vertices_end(),
                      Point_2(rect.xmax()+0.1, rect.ymin()));
    right_wall.insert(right_wall.vertices_end(),
                      Point_2(rect.xmax(), rect.ymin()));
    right_wall.insert(right_wall.vertices_end(),
                      Point_2(rect.xmax(), rect.ymax()));
    right_wall.insert(right_wall.vertices_end(),
                      Point_2(rect.xmax()+0.1, rect.ymax()));
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

////
// Code from cgal example
////
void insert_polygon(CDT& cdt,const Polygon_2& polygon){
  if ( polygon.is_empty() ) return;
  CDT::Vertex_handle v_prev=cdt.insert(*CGAL::cpp11::prev(polygon.vertices_end()));
  for (Polygon_2::Vertex_iterator vit=polygon.vertices_begin();
       vit!=polygon.vertices_end();++vit)
  {
    CDT::Vertex_handle vh=cdt.insert(*vit);
    cdt.insert_constraint(vh,v_prev);
    v_prev=vh;
  }  
}
////
// End of example code
////

void WallMap::BuildTriangles() {
  double start_time_sec = ros::Time::now().toSec();

  ROS_INFO("WallMap: Building Triangles");
  _triangles.clear();
  for (Polygon_2 raw_poly : _raw_obstacles) {
    ROS_INFO_ONCE("WallMap: Adding a triangulation of an obstacle");

    CDT cdt;
    insert_polygon(cdt, raw_poly);
    ROS_ASSERT(cdt.is_valid());
    for (CDT::Finite_faces_iterator fit=cdt.finite_faces_begin();
                                    fit!=cdt.finite_faces_end();++fit) {
      _triangles.push_back(cdt.triangle(fit));
    }
  }
  ROS_INFO("WallMap: Adding %d triangles took: %.3lf sec.",
      int(_triangles.size()), ros::Time::now().toSec()-start_time_sec);
  ROS_INFO("WallMap: Done Building Triangulation!");
}

double WallMap::DistanceToWall(const Ray_2 &ray) {
  double closest = pow(SONAR_MAX_RANGE, 2.0);
  int interCount = _triangles.size();
  for (const auto &tri : _triangles) {
    //ROS_INFO_STREAM("Querying: " << tri << " ray: " << ray);
    auto result = CGAL::intersection(ray, tri);
    if (const Point_2 *ipoint = CGAL::object_cast<Point_2>(&result)) {
      closest = min(closest, CGAL::to_double(CGAL::squared_distance(ray.source(), *ipoint)));
    } else if (const Segment_2 *iseg = CGAL::object_cast<Segment_2>(&result)) {
      closest = min(closest, CGAL::to_double(CGAL::squared_distance(ray.source(), iseg->source())));
      closest = min(closest, CGAL::to_double(CGAL::squared_distance(ray.source(), iseg->target())));
    } else {
      --interCount;
    }
  }
  ROS_ASSERT(interCount > 0);
  return sqrt(closest);
}

} // namespace localization 
