#ifndef __LOCALIZATION_WALLMAP_H__
#define __LOCALIZATION_WALLMAP_H__

#include <ros/ros.h>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/point_generators_3.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polyhedron_3.h>
#include <fstream>

typedef CGAL::Exact_predicates_exact_constructions_kernel cgal_kernel;
//typedef CGAL::Simple_cartesian<double> cgal_kernel;

namespace localization {

class WallMap {
 public:
  static constexpr double SONAR_MAX_RANGE = 5.0;

  WallMap(const std::string &mapfile_location);

 public:
  /***
   * Helper functions.
   **/
  double DistanceToWall(const cgal_kernel::Ray_2 &ray);

  const cgal_kernel::Point_2 GetRobotStart() const {
    return _robot_start;
  }

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

  void BuildTriangles();

 private:
  cgal_kernel::Iso_rectangle_2 _world_rect;
  std::vector< CGAL::Polygon_2<cgal_kernel>> _raw_obstacles;
  std::vector<std::shared_ptr<CGAL::Polyhedron_3<cgal_kernel>>> _obs_polyhedra;

 public:
  cgal_kernel::Point_2 _robot_start;
  std::vector< CGAL::Triangle_2<cgal_kernel>> _triangles;

};

} // namespace localization 

#endif // __LOCALIZATION_WALLMAP_H__
