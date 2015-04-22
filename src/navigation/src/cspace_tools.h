#ifndef __NAVIGATION_CSPACETOOLS_H__
#define __NAVIGATION_CSPACETOOLS_H__

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_2.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel cgal_kernel_exact;
typedef CGAL::Exact_predicates_exact_constructions_kernel cgal_kernel;
//typedef CGAL::Simple_cartesian<double> cgal_kernel;

namespace cspace {

void GetCSpaceObstacle2d(
    const CGAL::Polygon_2<cgal_kernel> &poly,
    const int rotInd,
    CGAL::Polygon_2<cgal_kernel> *cspoly);

CGAL::Polygon_2<cgal_kernel_exact> GetReflectedRobotRepresentation(
      double rotation);


} // namespace cspace 

#endif // __NAVIGATION_CSPACETOOLS_H__
