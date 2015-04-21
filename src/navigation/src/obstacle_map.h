#ifndef __NAVIGATION_OBSTACLEMAP_H__
#define __NAVIGATION_OBSTACLEMAP_H__

#include "ros/ros.h"

#include <CGAL/AABB_polyhedron_triangle_primitive.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/point_generators_3.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Simple_cartesian.h>
#include <fstream>

namespace cspace {

class ObstacleMap {
 public:
  static const int ANGLE_DIVISIONS = 100;

  ObstacleMap(const std::string &mapfile_location);

 public:
  /***
   * Helper functions.
   **/
  double GetWorldWidth();
  double GetWorldHeight();
  double GetWorldCenterX();
  double GetWorldCenterY();
  bool WorldContains(const CGAL::Simple_cartesian<double>::Point_2 &point);
  bool WorldContains(const CGAL::Simple_cartesian<double>::Point_3 &point);
  static int RotationToId(double rotation);
  static double IdToRotation(int id);
  static double IdToRad(int id);
  static double RadToRotation(double rad);
  bool IsObstacleFree(const CGAL::Simple_cartesian<double>::Point_3 &point,
                      const double radius);
  bool IsInsideObstacle(const CGAL::Simple_cartesian<double>::Point_3 &point);
  double DistanceToClosestObstacle(
      const CGAL::Simple_cartesian<double>::Point_3 &point);

 private:
  void BuildMapFromFile(const std::string &mapfile_location); 
  void ParseFromFile(const std::string &mapfile_location);
  void ParsePoint(
      std::ifstream &stream,
      CGAL::Simple_cartesian<double>::Point_2 *point);
  void ParseRect(
      std::ifstream &stream,
      CGAL::Simple_cartesian<double>::Iso_rectangle_2 *point);
  bool ParseObstacle(
      std::ifstream &stream,
      CGAL::Polygon_2<CGAL::Simple_cartesian<double>> *poly);

  void BuildCSpace();

 private:
  CGAL::Simple_cartesian<double>::Point_2 _robot_start;
  CGAL::Simple_cartesian<double>::Point_2 _robot_goal;
  CGAL::Simple_cartesian<double>::Iso_rectangle_2 _world_rect;
  std::vector< CGAL::Polygon_2<CGAL::Simple_cartesian<double> > > _raw_obstacles;
  CGAL::AABB_tree<
    CGAL::AABB_traits<
      CGAL::Simple_cartesian<double>,
      CGAL::AABB_polyhedron_triangle_primitive<
        CGAL::Simple_cartesian<double>,
        CGAL::Polyhedron_3<CGAL::Simple_cartesian<double> >
      >
    >
  > _obstacles_tree;
  std::unique_ptr<
    CGAL::Random_points_on_sphere_3<
      CGAL::Simple_cartesian<double>::Point_3>> _point_gen;

};

} // namespace cspace 

#endif // __NAVIGATION_OBSTACLEMAP_H__
