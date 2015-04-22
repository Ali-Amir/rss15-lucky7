#include "grid.h"

using namespace std;

typedef cgal_kernel K;
typedef CGAL::Point_2<K> Point_2;
typedef CGAL::Point_3<K> Point_3;

namespace cspace {

int Grid::BFS_COLOR;

Grid::Grid(const shared_ptr<ObstacleMap> &obs_map) :
    _obs_map(obs_map) {
  ROS_INFO("Initializing grid");
  _cells = new Cell[ObstacleMap::ANGLE_DIVISIONS*ROWS*COLS];
  _resolutionX = 2.0*_obs_map->GetWorldWidth()/ROWS;
  _resolutionY = 2.0*_obs_map->GetWorldHeight()/COLS;

  double world_centerX = _obs_map->GetWorldCenterX();
  double world_centerY = _obs_map->GetWorldCenterY();
  double cell_radius = sqrt(_resolutionX*_resolutionX +
                            _resolutionY*_resolutionY);
  ROS_INFO("Allocated memory and created necessary vars.");
  int impassable = 0, passable = 0;
  for (int rotId = 0; rotId < ObstacleMap::ANGLE_DIVISIONS; ++rotId) {
    //ROS_INFO("Adding cells on level %d", rotId);
    double ux = cos(ObstacleMap::IdToRad(rotId));
    double uy = sin(ObstacleMap::IdToRad(rotId));
    double vx = cos(ObstacleMap::IdToRad(rotId)+M_PI/2.0);
    double vy = sin(ObstacleMap::IdToRad(rotId)+M_PI/2.0);
    double cur_time = ros::Time::now().toSec();
    for (int r = 0; r < ROWS; ++r) {
      for (int c = 0; c < COLS; ++c) {
        int dr = (ROWS >> 1) - r, dc = (COLS >> 1) - c;
        double centerX = world_centerX + (dc*ux + dr*vx)*_resolutionX;
        double centerY = world_centerY + (dc*uy + dr*vy)*_resolutionY;
        //ROS_INFO("Here1 %d/%d", CellId::GetCellId(r, c, rotId), ROWS*COLS*ObstacleMap::ANGLE_DIVISIONS);
        _cells[CellId::GetCellId(r, c, rotId)] =
            Cell(r, c, rotId, centerX, centerY, _resolutionX, _resolutionY);
        //ROS_INFO("point: %.3lf %.3lf %.3lf", centerX, centerY, ObstacleMap::IdToRotation(rotId));
        if (!_obs_map->WorldContains(Point_2(centerX, centerY)) ||
            _obs_map->IsInsideObstacle(Point_2(centerX, centerY), rotId)) {
//                                              ObstacleMap::IdToRotation(rotId)),
//                                      cell_radius)) {
          ++impassable;
          _cells[CellId::GetCellId(r, c, rotId)].is_free = false;
        } else {
          ++passable;
          _free_cells.push_back(CellId(r, c, rotId));
        }
      }
    }
    ROS_INFO_THROTTLE(10, "Marking level: %d took: %.3lf sec.", rotId, (ros::Time::now().toSec()-cur_time));
  }
  ROS_INFO_STREAM("Done marking obstacles. " <<
                  "Impassable cells: " << impassable <<
                  ". Passable cells: " << passable);
}

double Grid::ComputePathsToGoal(const Point_2 &goal) {
  vector<Point_3> goal_points;
  for (int rotId = 0; rotId < ObstacleMap::ANGLE_DIVISIONS; ++rotId) {
    goal_points.push_back(Point_3(goal.x(), goal.y(),
                                  ObstacleMap::IdToRotation(rotId)));
  }
  ROS_INFO("Calling compute paths to goal given a vector of points.");
  return ComputePathsToGoal(goal_points);
}

double Grid::ComputePathsToGoal(const Point_3 &goal) {
  vector<Point_3> goal_points;
  goal_points.push_back(goal);
  return ComputePathsToGoal(goal_points);
}

double Grid::ComputePathsToGoal(const vector<Point_3> &points) {
  vector<CellId> start_ids;
  ROS_INFO("Getting the list of cell ids.");
  for (const Point_3& goal : points) {
    CellId goal_cell_id;
    if (!GetCellId(goal, &goal_cell_id)) {
      return numeric_limits<double>::infinity();
    }
   
    start_ids.push_back(goal_cell_id);
  }
  // Check if we already did the same computation in previous step.
  sort(start_ids.begin(), start_ids.end());
  ROS_INFO("Got the list of cell ids.");
  if (start_ids == _previous_start_ids) {
    return -1.0;
  }
  ROS_INFO("Calling bfs.");
  _previous_start_ids = start_ids;
  return RunBfs(start_ids);
}

double Grid::RunBfs(vector<CellId> start_ids) {
  queue<CellId> queue;
  ++BFS_COLOR;
  ROS_INFO("Starting bfs");
  for (const CellId &cell_id : start_ids) {
    assert(cell_id.r < ROWS && cell_id.c < COLS && cell_id.rotId < ObstacleMap::ANGLE_DIVISIONS);
    /*
    ROS_INFO("Marking cell number: %d/%d", cell_id.GetIndex(), ROWS*COLS*ObstacleMap::ANGLE_DIVISIONS);
    ROS_INFO("Cell is: r:%d/%d c:%d/%d rotId:%d/%d", cell_id.r, ROWS, cell_id.c, COLS, cell_id.rotId, ObstacleMap::ANGLE_DIVISIONS);
    */
    _cells[cell_id.GetIndex()].color = BFS_COLOR;
    _cells[cell_id.GetIndex()].min_dist_to_goal = 0.0;
    _cells[cell_id.GetIndex()].to_goal_next = nullptr;
    queue.push(cell_id);
  }

  ROS_INFO("Doing bfs!");
  double max_dist = 0.0;
  vector<CellId> neighbor_ids;
  while (!queue.empty()) {
    CellId cur_cell_id = queue.front();
    queue.pop();
    assert(cur_cell_id.GetIndex() < ROWS*COLS*ObstacleMap::ANGLE_DIVISIONS);

    double cur_dist = _cells[cur_cell_id.GetIndex()].min_dist_to_goal;
    ROS_INFO_THROTTLE(5, "Collecting neighbors for vertex: %d", cur_cell_id.GetIndex());
    CollectNeighbors(cur_cell_id, &neighbor_ids);
    ROS_INFO_THROTTLE(5, "Got neighbors!");
    for (const CellId &next_id : neighbor_ids) {
      assert(next_id.GetIndex() < ROWS*COLS*ObstacleMap::ANGLE_DIVISIONS);
      if (_cells[next_id.GetIndex()].is_free &&
          _cells[next_id.GetIndex()].color != BFS_COLOR) {
        _cells[next_id.GetIndex()].min_dist_to_goal = cur_dist + _resolutionX; 
        _cells[next_id.GetIndex()].to_goal_next = _cells + cur_cell_id.GetIndex();
        _cells[next_id.GetIndex()].color = BFS_COLOR;
        max_dist = max(max_dist, cur_dist + _resolutionX);
        queue.push(next_id);
      }
    }
  }
  ROS_INFO("Done doing bfs!");
  return max_dist;
}

void Grid::CollectNeighbors(const CellId &ver, vector<CellId> *neighbors) {
  neighbors->clear();

  //below
  if (ver.r > 0)
    neighbors->push_back(CellId(ver.r-1, ver.c, ver.rotId));

  //above
  if (ver.r < ROWS-1)
    neighbors->push_back(CellId(ver.r+1, ver.c, ver.rotId));

  //left
  if (ver.c > 0)
    neighbors->push_back(CellId(ver.r, ver.c-1, ver.rotId));

  //right
  if (ver.c < COLS-1)
    neighbors->push_back(CellId(ver.r, ver.c+1, ver.rotId));

  if (ver.rotId + 1 < ObstacleMap::ANGLE_DIVISIONS) {
    neighbors->push_back(CellId(ver.r, ver.c, ver.rotId+1));
  } else {
    neighbors->push_back(CellId(ver.r, ver.c, 0));
  }

  if (ver.rotId > 0) {
    neighbors->push_back(CellId(ver.r, ver.c, ver.rotId-1));
  } else {
    neighbors->push_back(CellId(ver.r, ver.c, ObstacleMap::ANGLE_DIVISIONS-1));
  }
}

bool Grid::GetCellId(const Point_3 &point, CellId *cell_id) {
  if (!_obs_map->WorldContains(point)) {
    return false;
  }

  double world_centerX = _obs_map->GetWorldCenterX();
  double world_centerY = _obs_map->GetWorldCenterY();
  int rotId = ObstacleMap::RotationToId(CGAL::to_double(point.z()));

  double ux = cos(ObstacleMap::IdToRad(rotId));
  double uy = sin(ObstacleMap::IdToRad(rotId));
  double vx = cos(ObstacleMap::IdToRad(rotId)+M_PI/2.0);
  double vy = sin(ObstacleMap::IdToRad(rotId)+M_PI/2.0);

  // Solve system of linear equations:
  // dc*ux + dr*vx = varX
  // dc*uy + dr*vy = varY
  double varX = CGAL::to_double(point.x()-world_centerX)/_resolutionX;
  double varY = CGAL::to_double(point.y()-world_centerY)/_resolutionY;
  double detA = (ux*vy - uy*vx);

  int dc = (int)((varX*vy - varY*vx)/detA);
  int dr = (int)((ux*varY - uy*varX)/detA);

  int actualC = max(0, min(COLS-1, (COLS>>1) - dc));
  int actualR = max(0, min(ROWS-1, (ROWS>>1) - dr));

  *cell_id = CellId(actualR, actualC, rotId);
  return true;
}

} // namespace cspace 
