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
typedef Polygon_2::Edge_const_iterator EdgeIterator;


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

  FILE *fin;
  fin = fopen(mapfile_location.c_str(), "r");

  ifstream mapstream;
  mapstream.open(mapfile_location, ifstream::in);
  ParsePoint(fin, &_robot_start);
  Point_2 tmp_point;
  ParsePoint(fin, &tmp_point);
  Iso_rectangle_2 rect;
  ParseRect(fin, &rect);

  _raw_obstacles.clear();
  int numObstacles;
  ROS_ASSERT(fscanf(fin, "%d", &numObstacles) == 1);
  for (int obstacleNumber = 0; obstacleNumber < numObstacles;
       ++obstacleNumber) {
    Polygon_2 poly;
    if (!ParseObstacle(fin, &poly)) {
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
  fclose(fin);
}

/***
 * Parses a 2d point from input stream.
 **/
void WallMap::ParsePoint(FILE *fin, Point_2 *point) {
  double x, y;
  ROS_ASSERT(fscanf(fin, "%lf%lf", &x, &y) == 2);
  *point = Point_2(x, y);
}

/***
 * Parses world boundary rectangle from input stream.
 **/
void WallMap::ParseRect(FILE *fin, Iso_rectangle_2 *rect) {
  double x, y, size_x, size_y;
  ROS_ASSERT(fscanf(fin, "%lf%lf%lf%lf", &x, &y, &size_x, &size_y)==4);
  Point_2 corner(x, y), size(size_x, size_y);
  *rect = Iso_rectangle_2(corner, corner+Vector_2(size.x(), size.y()));
}

/***
 * Parses raw obstacles from input stream.
 **/
bool WallMap::ParseObstacle(FILE *fin, Polygon_2 *poly) {
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
  double closest = SONAR_MAX_RANGE;
  double x = CGAL::to_double(ray.source().x());
  double y = CGAL::to_double(ray.source().y());
  double vx = CGAL::to_double(ray.direction().dx());
  double vy = CGAL::to_double(ray.direction().dy());
  double lv = sqrt(vx*vx+vy*vy);
  vx /= lv;
  vy /= lv;

  int interCount = 0;

  for (const Polygon_2 raw_poly : _raw_obstacles) {
    for (EdgeIterator ei = raw_poly.edges_begin();
         ei != raw_poly.edges_end(); ++ei) {
      double x1 = CGAL::to_double(ei->source().x());
      double y1 = CGAL::to_double(ei->source().y());
      double x2 = CGAL::to_double(ei->target().x());
      double y2 = CGAL::to_double(ei->target().y());
      double A = y2-y1;
      double B = x1-x2;
      double C = x2*y1-x1*y2;
      if (fabs(A*vx+B*vy) < 1e-6) {
        if (fabs(A*x + B*y + C) < 1e-6) {
          double dis1 = vx*(x1-x)+vy*(y1-y);
          if (dis1 > 0) {
            closest = min(closest, dis1);
            ++interCount;
          }
          double dis2 = vx*(x2-x)+vy*(y2-y);
          if (dis2 > 0) {
            closest = min(closest, dis2);
            ++interCount;
          }
        }
      } else {
        double t = (-C - A*x - B*y) / (A*vx + B*vy);
        if (t >= 0) {
          double xc = x+vx*t, yc = y+vy*t;
          if ((x1-xc)*(x2-xc)+(y1-yc)*(y2-yc) < 1e-6) {
            closest = min(closest, t);
            ++interCount;
          }
        }
      }
    }
  }
  ROS_ASSERT(interCount > 0);
  return closest;
}
/*
double WallMap::DistanceToWall(const Ray_2 &ray) {
  double closest = pow(SONAR_MAX_RANGE, 2.0);
  int interCount = _triangles.size();
  Point_2 ipoint;
  Segment_2 iseg;
  for (const auto &tri : _triangles) {
    //ROS_INFO_STREAM("Querying: " << tri << " ray: " << ray);
    CGAL::Object result = CGAL::intersection(ray, tri);
    if (CGAL::assign(ipoint, result)) {
      closest = min(closest, CGAL::to_double(CGAL::squared_distance(ray.source(), ipoint)));
    } else if (CGAL::assign(iseg, result)) {
      closest = min(closest, CGAL::to_double(CGAL::squared_distance(ray.source(), iseg.source())));
      closest = min(closest, CGAL::to_double(CGAL::squared_distance(ray.source(), iseg.target())));
    } else {
      --interCount;
    }
  }
  ROS_ASSERT(interCount > 0);
  return sqrt(closest);
}
*/

} // namespace localization 
