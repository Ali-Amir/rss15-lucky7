#ifndef __NAVIGATION_OBSTACLEMAP_H__
#define __NAVIGATION_OBSTACLEMAP_H__

#include <ros/ros.h>

#include <CGAL/AABB_polyhedron_triangle_primitive.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/point_generators_3.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polyhedron_3.h>
#include <fstream>

#include "cspace_tools.h"

namespace cspace {

class ObstacleMap {
 public:
  static const int ANGLE_DIVISIONS = 120;

  ObstacleMap(const std::string &mapfile_location);

 public:
  /***
   * Helper functions.
   **/
  double GetWorldWidth();
  double GetWorldHeight();
  double GetWorldCenterX();
  double GetWorldCenterY();
  bool WorldContains(const cgal_kernel::Point_2 &point);
  bool WorldContains(const cgal_kernel::Point_3 &point);
  static int RadToId(double rad);
  static int RotationToId(double rotation);
  static double IdToRotation(int id);
  static double IdToRad(int id);
  static double RadToRotation(double rad);
  static double RotationToRad(double rot);
  bool IsObstacleFree(const cgal_kernel::Point_3 &point,
                      const double radius);
  bool IsInsideObstacle(const cgal_kernel::Point_2 &point);
  bool IsInsideObstacle(const cgal_kernel::Point_3 &point);
  bool IsInsideObstacle(const cgal_kernel::Point_2 &point,
                        const int rotInd);
  double DistanceToClosestObstacle(
      const cgal_kernel::Point_3 &point);

 private:
  void BuildMapFromFile(const std::string &mapfile_location); 
  void ParseFromFile(const std::string &mapfile_location);
  void ParsePoint(
      FILE *fin,
      cgal_kernel::Point_2 *point);
  void ParseRect(
      FILE *fin,
      cgal_kernel::Iso_rectangle_2 *point);
  bool ParseObstacle(
      FILE *fin,
      CGAL::Polygon_2<cgal_kernel> *poly);

  void BuildCSpace();

 public:
  cgal_kernel::Point_2 _robot_start;
  cgal_kernel::Point_2 _robot_goal;
 private:
  cgal_kernel::Iso_rectangle_2 _world_rect;
  std::vector< CGAL::Polygon_2<cgal_kernel > > _raw_obstacles;
  CGAL::AABB_tree<
    CGAL::AABB_traits<
      cgal_kernel,
      CGAL::AABB_polyhedron_triangle_primitive<
        cgal_kernel,
        CGAL::Polyhedron_3<cgal_kernel>
      >
    >
  > _obstacles_tree;
  std::unique_ptr<
    CGAL::Random_points_on_sphere_3<
      cgal_kernel::Point_3>> _point_gen;
  std::vector<std::shared_ptr<CGAL::Polyhedron_3<cgal_kernel>>> _obs_polyhedra;
 public:
  std::vector<std::shared_ptr<CGAL::Polygon_2<cgal_kernel>>>
      _lvl_obstacles[ANGLE_DIVISIONS];

};

} // namespace cspace 

#endif // __NAVIGATION_OBSTACLEMAP_H__
