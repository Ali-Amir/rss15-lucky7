#ifndef __NAVIGATION_CSPACETOOLS_H__
#define __NAVIGATION_CSPACETOOLS_H__

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_2.h>

namespace cspace {

void GetCSpaceObstacle2d(
    const CGAL::Polygon_2<CGAL::Simple_cartesian<double>> &poly,
    const int rotInd,
    CGAL::Polygon_2<CGAL::Simple_cartesian<double>> *cspoly);

CGAL::Polygon_2<
  CGAL::Simple_cartesian<double>> GetReflectedRobotRepresentation(
      double rotation);


} // namespace cspace 

#endif // __NAVIGATION_CSPACETOOLS_H__
