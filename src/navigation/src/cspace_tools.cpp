#include "cspace_tools.h"

#include "obstacle_map.h"

#include <CGAL/minkowski_sum_2.h>
#include <CGAL/ch_graham_andrew.h>

using namespace std;

namespace cspace {

typedef CGAL::Simple_cartesian<double> K;
typedef K::Segment_3 Segment_3;
typedef K::Point_2 Point_2;
typedef K::Point_3 Point_3;
typedef K::Iso_rectangle_2 Iso_rectangle_2;
typedef K::Vector_2 Vector_2;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef CGAL::Polygon_with_holes_2<K> Polygon_with_holes_2;


void GetCSpaceObstacle2d(const Polygon_2 &poly, const int rotInd,
                         Polygon_2 *cspoly) {
  double rotation = rotInd*360.0/ObstacleMap::ANGLE_DIVISIONS;
  Polygon_2 reflected_robot = GetReflectedRobotRepresentation(rotation);
  Polygon_with_holes_2 sum = minkowski_sum_2(reflected_robot, poly);
  *cspoly = sum.outer_boundary();
}

Polygon_2 GetReflectedRobotRepresentation(double rotation) {
  double pForward = 0.21, pBackward = -0.35,
         pLeft = 0.25, pRight = -0.25;
  vector<Point_2> points;
  points.push_back(
    Point_2(
      -(pForward*cos(rotation)-pLeft*sin(rotation)),
      -(pForward*sin(rotation)+pLeft*cos(rotation))));
  points.push_back(
    Point_2(
      -(pForward*cos(rotation)-pRight*sin(rotation)),
      -(pForward*sin(rotation)+pRight*cos(rotation))));
  points.push_back(
    Point_2(
      -(pBackward*cos(rotation)-pLeft*sin(rotation)),
      -(pBackward*sin(rotation)+pLeft*cos(rotation))));
  points.push_back(
    Point_2(
      -(pBackward*cos(rotation)-pRight*sin(rotation)),
      -(pBackward*sin(rotation)+pRight*cos(rotation))));

  vector<Point_2> hull;
  CGAL::ch_graham_andrew(points.begin(), points.end(), back_inserter(hull));

  Polygon_2 reflected_robot;
  for (Point_2 p : hull) {
    reflected_robot.insert(reflected_robot.vertices_end(), p);
  }
  return reflected_robot;
}

} // namespace cspace 
