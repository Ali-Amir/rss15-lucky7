#include "cspace_tools.h"

#include "obstacle_map.h"

#include <CGAL/minkowski_sum_2.h>
#include <CGAL/ch_graham_andrew.h>

using namespace std;

namespace cspace {

typedef cgal_kernel K;
typedef K::Segment_3 Segment_3;
typedef K::Point_2 Point_2;
typedef K::Point_3 Point_3;
typedef K::Iso_rectangle_2 Iso_rectangle_2;
typedef K::Vector_2 Vector_2;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef CGAL::Polygon_with_holes_2<cgal_kernel_exact> Polygon_with_holes_2_exact;
typedef CGAL::Point_2<cgal_kernel_exact> Point_2_exact;
typedef CGAL::Polygon_2<cgal_kernel_exact> Polygon_2_exact;


void GetCSpaceObstacle2d(const Polygon_2 &poly, const int rotInd,
                         Polygon_2 *cspoly) {
  double rotation = ObstacleMap::IdToRotation(rotInd);
  Polygon_2_exact reflected_robot = GetReflectedRobotRepresentation(rotation);
  Polygon_2_exact poly_exact;
  for (int i = 0; i < poly.size(); ++i) {
    poly_exact.insert(poly_exact.vertices_end(), Point_2_exact(poly[i].x(), poly[i].y()));
  }
  Polygon_with_holes_2_exact sum = minkowski_sum_2(reflected_robot, poly_exact);
  Polygon_2_exact cspoly_exact = sum.outer_boundary();
  cspoly->clear();
  for (int i = 0; i < cspoly_exact.size(); ++i) {
    cspoly->insert(cspoly->vertices_end(), Point_2(CGAL::to_double(cspoly_exact[i].x()), CGAL::to_double(cspoly_exact[i].y())));
  }
}

Polygon_2_exact GetReflectedRobotRepresentation(double rotation) {
  double pForward = 0.21, pBackward = -0.35,
         pLeft = 0.25, pRight = -0.25;
  vector<Point_2_exact> points;
  points.push_back(
    Point_2_exact(
      -(pForward*cos(rotation)-pLeft*sin(rotation)),
      -(pForward*sin(rotation)+pLeft*cos(rotation))));
  points.push_back(
    Point_2_exact(
      -(pForward*cos(rotation)-pRight*sin(rotation)),
      -(pForward*sin(rotation)+pRight*cos(rotation))));
  points.push_back(
    Point_2_exact(
      -(pBackward*cos(rotation)-pLeft*sin(rotation)),
      -(pBackward*sin(rotation)+pLeft*cos(rotation))));
  points.push_back(
    Point_2_exact(
      -(pBackward*cos(rotation)-pRight*sin(rotation)),
      -(pBackward*sin(rotation)+pRight*cos(rotation))));

  vector<Point_2_exact> hull;
  CGAL::ch_graham_andrew(points.begin(), points.end(), back_inserter(hull));

  Polygon_2_exact reflected_robot;
  for (Point_2_exact p : hull) {
    reflected_robot.insert(reflected_robot.vertices_end(), p);
  }
  return reflected_robot;
}

} // namespace cspace 
