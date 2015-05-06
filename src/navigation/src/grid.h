#ifndef __NAVIGATION_GRID_H__
#define __NAVIGATION_GRID_H__

#include "ros/ros.h"

#include "obstacle_map.h"
#include "cspace_tools.h"

#include <limits>
#include <memory>
#include <queue>

namespace cspace {

class Grid {
 public:
  static const int ROWS = 700;
  static const int COLS = 650;
  static const int ANGLE_DIVISIONS = ObstacleMap::ANGLE_DIVISIONS;
  static int BFS_COLOR;

  Grid(const std::shared_ptr<ObstacleMap> &obs_map);
  ~Grid() {
    delete[] _cells;
  }

  struct CellId {
    int r{-1};
    int c{-1};
    int rotId{-1};

    CellId() {}

    CellId(int r_, int c_, int rotId_) : r(r_), c(c_), rotId(rotId_) {}

    static int GetCellId(int _r, int _c, int _rotId) {
      return _rotId*(ROWS*COLS) + _r*COLS + _c;
    }

    int GetIndex() const {
      return rotId*(ROWS*COLS) + r*COLS + c;
    }

    bool operator < (const CellId &a) const {
      return GetIndex() < a.GetIndex();
    }

    bool operator == (const CellId &a) const {
      return GetIndex() == a.GetIndex();
    }
  };

  struct Cell {
    int r{-1};
    int c{-1};
    int rotId{-1};
    int color{-1};
    double xc;
    double yc;
    double half_width;
    double half_height;
    double min_dist_to_goal{std::numeric_limits<double>::infinity()};
    bool is_free{true};
    Cell *to_goal_next;

    Cell() {}

    Cell(int r_, int c_, int rotId_, double xc_, double yc_,
         double half_width_, double half_height_) :
      r(r_), c(c_), rotId(rotId_), xc(xc_), yc(yc_), half_width(half_width_),
      half_height(half_height_) {
    }

    int GetIndex() const {
      return CellId::GetCellId(r, c, rotId);
    }
    
    bool HasPathToGoal() const {
      return is_free && color == Grid::BFS_COLOR;
    }
  };

  std::vector<CellId> ComputeInitCells(
      const cgal_kernel::Point_2 &goal, int *status);
  std::vector<CellId> ComputeInitCells(
      const cgal_kernel::Point_3 &goal, int *status);
  std::vector<CellId> ComputeInitCells(
      const std::vector<cgal_kernel::Point_3> &points, int *status);
  double ComputePathsToGoal(
      const cgal_kernel::Point_2 &point, int *status);
  double ComputePathsToGoal(
      const cgal_kernel::Point_3 &point, int *status);
  double ComputePathsToGoal(
      const std::vector<cgal_kernel::Point_3> &point, int *status);

  double RunBfs(std::vector<CellId> start_ids);
  void CollectNeighbors(const CellId &ver, std::vector<CellId> *neighbors);

  bool GetCellId(const cgal_kernel::Point_3 &point,
                 CellId *cell_id);
  const Cell* GetCell(const CellId &cell_id) {
    if (_compressed_id.count(cell_id.GetIndex())) {
      return _cells + _compressed_id[cell_id.GetIndex()];
    }
    return nullptr;
  }

  const int DistToObs(const CellId &cell_id) {
    if (_compressed_id.count(cell_id.GetIndex())) {
      return _dist_to_obs[_compressed_id[cell_id.GetIndex()]];
    }
    return 0;
  }

  const std::vector<const Cell*> *getFreeCellsByRotId(int rotId) const {
    return &_free_cells_by_rotid[rotId];
  }

 private:
  Cell *_cells;
  std::shared_ptr<ObstacleMap> _obs_map;
  double _resolutionX;
  double _resolutionY;
  std::vector<Cell> _free_cells;
  std::map<int,int> _compressed_id;
  std::vector<std::vector<int>> _edge;
  std::vector<const Cell*> _free_cells_by_rotid[ANGLE_DIVISIONS];
  std::vector<int> _dist_to_obs;
  std::vector<int> _dp;
 public:
  std::vector<CellId> _previous_start_ids;
};

} // namespace cspace 

#endif // __NAVIGATION_GRID_H__
