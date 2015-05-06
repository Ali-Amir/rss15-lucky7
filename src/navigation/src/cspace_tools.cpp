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
  double rad = ObstacleMap::IdToRad(rotInd);
  Polygon_2_exact reflected_robot = GetReflectedRobotRepresentation(rad);
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

Polygon_2_exact GetReflectedRobotRepresentation(double rad) {
  /*
  double pForward = 0.20, pBackward = -0.34,
         pLeft = 0.24, pRight = -0.24;
  */
  vector<Point_2_exact> points;
/*  points.push_back(
    Point_2_exact(
      -(pForward*cos(rad)-pLeft*sin(rad)),
      -(pForward*sin(rad)+pLeft*cos(rad))));
  points.push_back(
    Point_2_exact(
      -(pForward*cos(rad)-pRight*sin(rad)),
      -(pForward*sin(rad)+pRight*cos(rad))));
  points.push_back(
    Point_2_exact(
      -(pBackward*cos(rad)-pLeft*sin(rad)),
      -(pBackward*sin(rad)+pLeft*cos(rad))));
  points.push_back(
    Point_2_exact(
      -(pBackward*cos(rad)-pRight*sin(rad)),
      -(pBackward*sin(rad)+pRight*cos(rad))));
      */
  points.push_back(Point_2_exact(-0.27, 0.10));
  points.push_back(Point_2_exact(-0.27+0.15, 0.10+0.15));
  points.push_back(Point_2_exact(+0.27-0.14, 0.10+0.15));
  points.push_back(Point_2_exact(+0.27, 0.10));
  points.push_back(Point_2_exact(+0.27, -0.11));
  points.push_back(Point_2_exact(+0.23, -0.11));
  points.push_back(Point_2_exact(+0.23, -0.37));
  points.push_back(Point_2_exact(-0.23, -0.37));
  points.push_back(Point_2_exact(-0.23, -0.11));
  points.push_back(Point_2_exact(-0.27, -0.11));
  // Reflect and rotate
  for (int i = 0; i < points.size(); ++i) {
    double nx = CGAL::to_double(points[i].y());
    double ny = -CGAL::to_double(points[i].x());
    points[i] = Point_2_exact(-(nx*cos(rad)-ny*sin(rad)), -(nx*sin(rad)+ny*cos(rad)));
  }

  vector<Point_2_exact> hull;
  CGAL::ch_graham_andrew(points.begin(), points.end(), back_inserter(hull));

  Polygon_2_exact reflected_robot;
  for (Point_2_exact p : hull) {
    reflected_robot.insert(reflected_robot.vertices_end(), p);
  }
  return reflected_robot;
}

} // namespace cspace 
