#include "wall_map.h"

#include <CGAL/algorithm.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>

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
/*
typedef CGAL::Triangulation_vertex_base_2<K>                      Vb;
typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo2,K>    Fbb;
typedef CGAL::Constrained_triangulation_face_base_2<K,Fbb>        Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>               TDS;
typedef CGAL::Exact_predicates_tag                                Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag>  CDT;
typedef CDT::Point                                                Point;
typedef CGAL::Polygon_2<K>                                        Polygon_2;
*/

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
/*
void 
mark_domains(CDT& ct, 
             CDT::Face_handle start, 
             int index, 
             std::list<CDT::Edge>& border )
{
  if(start->info().nesting_level != -1){
    return;
  }
  std::list<CDT::Face_handle> queue;
  queue.push_back(start);
  while(! queue.empty()){
    CDT::Face_handle fh = queue.front();
    queue.pop_front();
    if(fh->info().nesting_level == -1){
      fh->info().nesting_level = index;
      for(int i = 0; i < 3; i++){
        CDT::Edge e(fh,i);
        CDT::Face_handle n = fh->neighbor(i);
        if(n->info().nesting_level == -1){
          if(ct.is_constrained(e)) border.push_back(e);
          else queue.push_back(n);
        }
      }
    }
  }
}
//explore set of facets connected with non constrained edges,
//and attribute to each such set a nesting level.
//We start from facets incident to the infinite vertex, with a nesting
//level of 0. Then we recursively consider the non-explored facets incident 
//to constrained edges bounding the former set and increase the nesting level by 1.
//Facets in the domain are those with an odd nesting level.
void
mark_domains(CDT& cdt)
{
  for(CDT::All_faces_iterator it = cdt.all_faces_begin(); it != cdt.all_faces_end(); ++it){
    it->info().nesting_level = -1;
  }
  std::list<CDT::Edge> border;
  mark_domains(cdt, cdt.infinite_face(), 0, border);
  while(! border.empty()){
    CDT::Edge e = border.front();
    border.pop_front();
    CDT::Face_handle n = e.first->neighbor(e.second);
    if(n->info().nesting_level == -1){
      mark_domains(cdt, n, e.first->info().nesting_level+1, border);
    }
  }
}
*/
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
    //cdt.insert_constraint(raw_poly.vertices_begin(), raw_poly.vertices_end(), true);
    ROS_ASSERT(cdt.is_valid());
    //mark_domains(cdt);
    for (CDT::Finite_faces_iterator fit=cdt.finite_faces_begin();
                                    fit!=cdt.finite_faces_end();++fit) {
      _triangles.push_back(cdt.triangle(fit));
    }
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
