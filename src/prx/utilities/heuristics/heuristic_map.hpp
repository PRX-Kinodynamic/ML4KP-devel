#pragma once
#include <queue>

#include "prx/utilities/defs.hpp"
#include "prx/planning/planners/rrt.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"

namespace prx
{
class heuristic_map_t
{
public:
  heuristic_map_t(double x_min, double x_max, double y_min, double y_max, double x_step, double y_step,
                  world_model_context context)
    : _x_min(x_min)
    , _y_min(y_min)
    , _x_max(x_max)
    , _y_max(y_max)
    , _x_step(x_step)
    , _y_step(y_step)
    , _context(context)
    , _u_att_coeff(1.0)
    , _u_rep_coeff(5.0)
  {
    rows = (int)((_y_max - _y_min) / _y_step);
    cols = (int)((_x_max - _x_min) / _x_step);
    std::cout << "Rows: " << rows << " Cols: " << cols << std::endl;

    obstacle_grid.resize(rows, std::vector<int>(cols, -1));

    dist_to_adjacent = std::sqrt(_x_step * _x_step + _y_step * _y_step);
  }

  void set_obstacle_grid()
  {
    for (int i = 0; i < rows; i++)
    {
      for (int j = 0; j < cols; j++)
      {
        _context.first->get_state_space()->at(0) = _x_min + j * _x_step;
        _context.first->get_state_space()->at(1) = _y_min + i * _y_step;
        obstacle_grid[i][j] = _context.second->in_collision() ? 0 : 1;
      }
    }
    brushfire();
    _obstacle_map_initialized = true;
  }

  double get_euclidean_distance(space_point_t point) const
  {
    return std::sqrt(std::pow(point->at(0) - _context.first->get_state_space()->at(0), 2) +
                     std::pow(point->at(1) - _context.first->get_state_space()->at(1), 2));
  }

  void set_heuristic_grid(space_point_t goal)
  {
    heuristic_grid.resize(rows, std::vector<double>(cols, -1));
    for (int i = 0; i < rows; i++)
    {
      for (int j = 0; j < cols; j++)
      {
        _context.first->get_state_space()->at(0) = _x_min + j * _x_step;
        _context.first->get_state_space()->at(1) = _y_min + i * _y_step;
        heuristic_grid[i][j] = _context.second->in_collision() ? 0 : get_euclidean_distance(goal);
      }
    }
    _heuristic_map_initialized = true;
  }

  void brushfire()
  {
    brushfire_grid.resize(rows, std::vector<double>(cols, -1));
    for (int i = 0; i < rows; i++)
    {
      for (int j = 0; j < cols; j++)
      {
        if (obstacle_grid[i][j] == 0)
        {
          brushfire_grid[i][j] = 1;
          bfs_queue.push({ i, j });
        }
        else if (obstacle_grid[i][j] == 1)
        {
          brushfire_grid[i][j] = 0;
        }
      }
    }

    while (!bfs_queue.empty())
    {
      auto [row, col] = bfs_queue.front();
      bfs_queue.pop();
      for (auto [n_row, n_col] : get_neighbors(row, col))
      {
        if (brushfire_grid[n_row][n_col] == 0)
        {
          brushfire_grid[n_row][n_col] = brushfire_grid[row][col] + dist_to_adjacent;
          bfs_queue.push({ n_row, n_col });
        }
      }
    }
  }
  
  double get_cost(space_point_t point) const
  {
    int row = (point->at(1) - _y_min) / _y_step;
    int col = (point->at(0) - _x_min) / _x_step;
    return get_cost(row, col);
  }

  double get_obstacle_cost(space_point_t point) const
  {
    int row = (point->at(1) - _y_min) / _y_step;
    int col = (point->at(0) - _x_min) / _x_step;
    if (!is_within_bounds(row, col))
    {
      return PRX_INFINITY;
    }
    return 1.0 / brushfire_grid[row][col];
  }

  double get_cost(int row, int col) const
  {
    if (!_obstacle_map_initialized || !_heuristic_map_initialized)
    {
      std::cerr << "Obstacle map or heuristic map not initialized!" << std::endl;
      return -1;
    }
    if (!is_within_bounds(row, col))
    {
      return PRX_INFINITY;
    }
    return _u_att_coeff * heuristic_grid[row][col] + _u_rep_coeff * (1.0 / brushfire_grid[row][col]);
    // return heuristic_grid[row][col] - brushfire_grid[row][col];
  }

  friend std::ostream& operator<<(std::ostream& os, const heuristic_map_t& map)
  {
    if (!map._obstacle_map_initialized || !map._heuristic_map_initialized)
    {
      os << "Obstacle map not initialized!" << std::endl;
      return os;
    }
    for (int i = 0; i < map.rows; i++)
    {
      for (int j = 0; j < map.cols; j++)
      {
        os << map.get_cost(i, j) << " ";
      }
      os << std::endl;
    }
    return os;
  }

  void set_u_att_coeff(double u_att_coeff)
  {
    _u_att_coeff = u_att_coeff;
  }

  void set_u_rep_coeff(double u_rep_coeff)
  {
    _u_rep_coeff = u_rep_coeff;
  }

private:
  int rows, cols;
  double _x_min, _y_min, _x_max, _y_max, _x_step, _y_step;
  double dist_to_adjacent;
  bool _obstacle_map_initialized = false, _heuristic_map_initialized = false;
  double _u_att_coeff, _u_rep_coeff;

  std::vector<std::vector<int>> obstacle_grid;
  std::vector<std::vector<double>> brushfire_grid;
  std::vector<std::vector<double>> heuristic_grid;
  std::queue<std::pair<int, int>> bfs_queue;

  world_model_context _context;
  space_point_t point;

  bool is_within_bounds(int row, int col) const
  {
    return row >= 0 && row < rows && col >= 0 && col < cols;
  }

  std::vector<std::pair<int, int>> get_neighbors(int row, int col) const
  {
    std::vector<std::pair<int, int>> neighbors;
    if (is_within_bounds(row - 1, col))
    {
      neighbors.push_back({ row - 1, col });
    }
    if (is_within_bounds(row + 1, col))
    {
      neighbors.push_back({ row + 1, col });
    }
    if (is_within_bounds(row, col - 1))
    {
      neighbors.push_back({ row, col - 1 });
    }
    if (is_within_bounds(row, col + 1))
    {
      neighbors.push_back({ row, col + 1 });
    }
    return neighbors;
  }
};
}  // namespace prx
