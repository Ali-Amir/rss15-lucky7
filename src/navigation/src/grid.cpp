#include "grid.h"

using namespace std;

typedef cgal_kernel K;
typedef CGAL::Point_2<K> Point_2;
typedef CGAL::Point_3<K> Point_3;

namespace cspace {

int Grid::BFS_COLOR;

Grid::Grid(const shared_ptr<ObstacleMap> &obs_map) :
    _obs_map(obs_map) {
  _resolutionX = 2.0*_obs_map->GetWorldWidth()/ROWS;
  _resolutionY = 2.0*_obs_map->GetWorldHeight()/COLS;
  ROS_INFO("Initializing grid: resolution (%.3lf, %.3lf)",
      _resolutionX, _resolutionY);

  ros::NodeHandle n;
  string precalc_path;
  ROS_ASSERT(n.getParam("/nav/precalcPath", precalc_path));
  string recalc_status;
  ROS_ASSERT(n.getParam("/nav/calculateFromScratch", recalc_status));

  bool have_precalc = 0;
  vector<bool> precalc_mask;
  if (recalc_status == "no") {
    ROS_INFO("Have precalc!");
    have_precalc = 1;

    precalc_mask.resize(ROWS*COLS*ANGLE_DIVISIONS);
    FILE *fin = fopen(precalc_path.c_str(), "r");
    for (int i = 0; i < ROWS*COLS*ANGLE_DIVISIONS; ++i) {
      int ch = fgetc(fin);
      ROS_ASSERT(ch != EOF);
      precalc_mask[i] = ch == '1';
    }
    int amo;
    ROS_ASSERT(fscanf(fin, "%d", &amo) == 1);
    _edge.resize(amo);
    for (int i = 0; i < amo; ++i) {
      int x;
      ROS_ASSERT(fscanf(fin, "%d", &x) == 1);
      for (; x--;) {
        int y;
        ROS_ASSERT(fscanf(fin, "%d", &y) == 1);
        _edge[i].push_back(y);
      }
    }
    _dist_to_obs.resize(amo);
    for (int i = 0; i < amo; ++i) {
      assert(fscanf(fin, "%d", &_dist_to_obs[i])==1);
    }
    fclose(fin);
  }

  ROS_INFO("Allocated memory and created necessary vars.");
  double world_centerX = _obs_map->GetWorldCenterX();
  double world_centerY = _obs_map->GetWorldCenterY();
  double cell_radius = sqrt(_resolutionX*_resolutionX +
                            _resolutionY*_resolutionY);
  int impassable = 0, passable = 0;
  for (int rotId = 0; rotId < ANGLE_DIVISIONS; ++rotId) {
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
        if (have_precalc) {
          if (precalc_mask[CellId::GetCellId(r, c, rotId)]) {
            ++passable;
            _free_cells.push_back(Cell(r, c, rotId, centerX, centerY,
                                       _resolutionX, _resolutionY));
          } else {
            ++impassable;
          }
        } else {
          int prev = (rotId - 1 + ANGLE_DIVISIONS) % ANGLE_DIVISIONS;
          int nxt = (rotId + 1) % ANGLE_DIVISIONS;
          if (!_obs_map->WorldContains(Point_2(centerX, centerY)) ||
              _obs_map->IsInsideObstacle(Point_2(centerX, centerY), rotId) ||
              _obs_map->IsInsideObstacle(Point_2(centerX, centerY), prev) ||
              _obs_map->IsInsideObstacle(Point_2(centerX, centerY), nxt)) {
            ++impassable;
          } else {
            ++passable;
            _free_cells.push_back(Cell(r, c, rotId, centerX, centerY,
                                       _resolutionX, _resolutionY));
          }
        }
      }
    }
    ROS_INFO_THROTTLE(10, "Marking level: %d took: %.3lf sec.", rotId, (ros::Time::now().toSec()-cur_time));
  }

  _cells = new Cell[_free_cells.size()];
  _compressed_id.clear();
  for (int i = 0; i < _free_cells.size(); ++i) {
    _cells[i] = _free_cells[i];
    _free_cells_by_rotid[_cells[i].rotId].push_back(_cells+i);
    _compressed_id[_free_cells[i].GetIndex()] = i;
  }

  ROS_INFO_STREAM("Done marking obstacles. " <<
                  "Impassable cells: " << impassable <<
                  ". Passable cells: " << passable);
  ROS_INFO_STREAM("Calculating neighbors now.");
  double cur_time = ros::Time::now().toSec();
  _dp.resize(_free_cells.size());
  if (!have_precalc) {
    _edge.clear();
    _dist_to_obs.clear();
    queue<int> que;
    for (int i = 0; i < _free_cells.size(); ++i) {
      _edge.push_back(vector<int>());
      CellId cur_cell(_cells[i].r, _cells[i].c, _cells[i].rotId);
      vector<CellId> neighbors;
      CollectNeighbors(cur_cell, &neighbors);
      _dist_to_obs.push_back(0);
      for (const CellId &next_cell : neighbors) {
        if (_compressed_id.count(next_cell.GetIndex())) {
          _edge[i].push_back(_compressed_id[next_cell.GetIndex()]);
        } else {
          _dist_to_obs[i] = 1;
        }
      }
      if (_dist_to_obs[i]) {
        que.push(i);
      }
    }
    while (que.size() > 0) {
      int u = que.front();
      que.pop();
      for (const int &v : _edge[u]) {
        if (!_dist_to_obs[v]) {
          _dist_to_obs[v] = _dist_to_obs[u]+1;
          que.push(v);
        }
      }
    }
  }
  ROS_INFO("Neighbors took: %.3lf sec.", ros::Time::now().toSec()-cur_time);

  if (!have_precalc) { 
    // Save results to file.
    FILE *precalc_fout = fopen(precalc_path.c_str(), "w");
    for (int i = 0; i < ROWS*COLS*ANGLE_DIVISIONS; ++i) {
      fputc(_compressed_id.count(i) ? '1' : '0', precalc_fout);
    }
    fprintf(precalc_fout, "\n%d\n", int(_edge.size()));
    fputc('\n', precalc_fout);
    for (int i = 0; i < _edge.size(); ++i) {
      fprintf(precalc_fout, "%d", int(_edge[i].size()));
      for (int e : _edge[i]) {
        fprintf(precalc_fout, " %d", e);
      }
      fprintf(precalc_fout, "\n");
    }
    for (int i = 0; i < _dist_to_obs.size(); ++i) {
      fprintf(precalc_fout, "%d ", _dist_to_obs[i]);
    }
    fprintf(precalc_fout, "\n");
    fclose(precalc_fout);
  }
}

vector<Grid::CellId> Grid::ComputeInitCells(const Point_2 &goal, int *status) {
  vector<Point_3> goal_points;
  for (int rotId = 0; rotId < ANGLE_DIVISIONS; ++rotId) {
    goal_points.push_back(Point_3(goal.x(), goal.y(),
                                  ObstacleMap::IdToRotation(rotId)));
  }
  return ComputeInitCells(goal_points, status);
}

vector<Grid::CellId> Grid::ComputeInitCells(const Point_3 &goal, int *status) {
  vector<Point_3> goal_points;
  goal_points.push_back(goal);
  return ComputeInitCells(goal_points, status);
}

vector<Grid::CellId> Grid::ComputeInitCells(const vector<Point_3> &points,
                                            int *status) {
  vector<CellId> start_ids;
  for (const Point_3& goal : points) {
    CellId goal_cell_id;
    if (!GetCellId(goal, &goal_cell_id)) {
      *status = 0;
      return vector<Grid::CellId>();
    }
   
    start_ids.push_back(goal_cell_id);
  }
  // Check if we already did the same computation in previous step.
  sort(start_ids.begin(), start_ids.end());
  if (start_ids == _previous_start_ids) {
    *status = 1;
  } else {
    *status = 0;
  }
  return start_ids;
}

double Grid::ComputePathsToGoal(const Point_2 &goal, int *status) {
  vector<Point_3> goal_points;
  for (int rotId = 0; rotId < ANGLE_DIVISIONS; ++rotId) {
    goal_points.push_back(Point_3(goal.x(), goal.y(),
                                  ObstacleMap::IdToRotation(rotId)));
  }
  ROS_DEBUG("Calling compute paths to goal given a vector of points.");
  return ComputePathsToGoal(goal_points, status);
}

double Grid::ComputePathsToGoal(const Point_3 &goal, int *status) {
  vector<Point_3> goal_points;
  goal_points.push_back(goal);
  return ComputePathsToGoal(goal_points, status);
}

double Grid::ComputePathsToGoal(const vector<Point_3> &points, int *status) {
  vector<CellId> start_ids;
  ROS_DEBUG("Getting the list of cell ids.");
  for (const Point_3& goal : points) {
    CellId goal_cell_id;
    if (!GetCellId(goal, &goal_cell_id)) {
      return numeric_limits<double>::infinity();
    }
   
    start_ids.push_back(goal_cell_id);
  }
  // Check if we already did the same computation in previous step.
  sort(start_ids.begin(), start_ids.end());
  ROS_DEBUG("Got the list of cell ids.");
  if (start_ids == _previous_start_ids) {
    *status = 1;
    return -1.0;
  }
  ROS_DEBUG("Calling bfs.");
  *status = 0;
  _previous_start_ids = start_ids;
  return RunBfs(start_ids);
}

double Grid::RunBfs(vector<CellId> start_ids) {
  priority_queue<pair<int,int>> queue;
  ++BFS_COLOR;
  ROS_INFO("Starting bfs: %d start ids", (int)start_ids.size());
  for (const CellId &cell_id : start_ids) {
    assert(cell_id.r < ROWS && cell_id.c < COLS && cell_id.rotId < ANGLE_DIVISIONS);
    if (!_compressed_id.count(cell_id.GetIndex())) {
      continue;
    }
    // TODO check if free
    /*
    ROS_INFO("Marking cell number: %d/%d", cell_id.GetIndex(), ROWS*COLS*ANGLE_DIVISIONS);
    ROS_INFO("Cell is: r:%d/%d c:%d/%d rotId:%d/%d", cell_id.r, ROWS, cell_id.c, COLS, cell_id.rotId, ANGLE_DIVISIONS);
    */
    int comp_ind = _compressed_id[cell_id.GetIndex()];
    _free_cells[comp_ind].color = BFS_COLOR;
    _cells[comp_ind].color = BFS_COLOR;
    _cells[comp_ind].min_dist_to_goal = 0.0;
    _cells[comp_ind].to_goal_next = nullptr;
    _dp[comp_ind] = 0;
    queue.push(make_pair(_dist_to_obs[comp_ind], comp_ind));
  }

  ROS_INFO("Doing bfs! Initial points: %d", (int)queue.size());
  int num_visited = 0;
  double max_dist = 0.0;
  //vector<CellId> neighbor_ids;
  while (!queue.empty()) {
    ++num_visited;
    int cur_cell = queue.top().second;
    int cur_dist = queue.top().first;
    /*
    if (cur_dist > _dp[cur_cell]) {
      continue;
    }
    */

    queue.pop();
    assert(cur_cell < ROWS*COLS*ANGLE_DIVISIONS);
    //ROS_ASSERT(_compressed_id.count(cur_cell_id.GetIndex()));
    //double cur_dist = _cells[cur_cell].min_dist_to_goal;
    for (const int &v : _edge[cur_cell]) {
      /*
      if (_cells[v].color != BFS_COLOR) {
        _dp[v] = _dp[cur_cell] + 2;
      }
      */

      int w = _dist_to_obs[v];
      if (_cells[v].color != BFS_COLOR) {
      //if (cur_dist + w < _dp[v]) {
        //_dp[v] = cur_dist - w;
        _cells[v].min_dist_to_goal = _cells[cur_cell].min_dist_to_goal
                                      + _resolutionX;
        _cells[v].to_goal_next = _cells + cur_cell;
        _free_cells[v].color = BFS_COLOR;
        _cells[v].color = BFS_COLOR;
        max_dist = max(max_dist, cur_dist + _resolutionX);
        queue.push(make_pair(w, v));
      }
    }
  }
  ROS_DEBUG("Done doing bfs! Visited: %d/%d cells.", num_visited, int(_compressed_id.size()));
  return max_dist;
}

void Grid::CollectNeighbors(const CellId &ver, vector<CellId> *neighbors) {
  neighbors->clear();

  //left
  if (ver.c > 0)
    neighbors->push_back(CellId(ver.r, ver.c-1, ver.rotId));

  //right
  if (ver.c < COLS-1)
    neighbors->push_back(CellId(ver.r, ver.c+1, ver.rotId));

  if (_compressed_id.count(ver.GetIndex())) {
    int ver_ind = _compressed_id[ver.GetIndex()];
    for (int i = ver.rotId-3; i < ver.rotId+4; ++i) {
      int id = (i + ANGLE_DIVISIONS)%ANGLE_DIVISIONS;
      if (ver.rotId != i) {
        CellId lvl_cell;
        GetCellId(Point_3(_cells[ver_ind].xc,
                          _cells[ver_ind].yc,
                          ObstacleMap::IdToRotation(id)),
                  &lvl_cell);
        neighbors->push_back(lvl_cell);
      }
    }
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

  int dc = (int)round((varX*vy - varY*vx)/detA);
  int dr = (int)round((ux*varY - uy*varX)/detA);

  int actualC = max(0, min(COLS-1, (COLS>>1) - dc));
  int actualR = max(0, min(ROWS-1, (ROWS>>1) - dr));

  *cell_id = CellId(actualR, actualC, rotId);
  return true;
}

} // namespace cspace 
