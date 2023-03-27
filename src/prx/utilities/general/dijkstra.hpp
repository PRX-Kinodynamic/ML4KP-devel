#pragma once
#include <queue>
#include <unordered_map>
#include "prx/utilities/defs.hpp"
namespace prx
{
namespace utilities
{

class dijkstra_t
{
public:
  template <typename Value, typename Cost>
  struct dijkstra_node_t
  {
    dijkstra_node_t(Value _value, Cost _cost) : value(_value), cost(_cost)
    {
    }
    friend bool operator<(dijkstra_node_t<Value, Cost> const& lhs, dijkstra_node_t<Value, Cost> const& rhs)
    {
      // PRX_DEBUG_VAR_3(lhs.cost, rhs.cost, (lhs.cost > rhs.cost));
      return (lhs.cost > rhs.cost);
    }
    friend bool operator>(dijkstra_node_t<Value, Cost> const& lhs, dijkstra_node_t<Value, Cost> const& rhs)
    {
      return !(lhs < rhs);
    }
    friend std::ostream& operator<<(std::ostream& os, const dijkstra_node_t<Value, Cost>& obj)
    {
      os << *(obj.value) << " " << obj.cost;
      return os;
    }
    Value value;
    Cost cost;
  };

  template <typename Value, typename Neighbors, typename NodeDistance, typename Cost = double>
  static std::vector<Value> shortest_path(const Value& source, const Value& to, const Neighbors& get_neighbors,
                                          const NodeDistance& distance_function)
  {
    using DjkNode = dijkstra_node_t<Value, Cost>;

    std::unordered_map<Value, double> distances;
    std::unordered_map<Value, Value> previous;

    distances[source] = 0;
    std::priority_queue<DjkNode, std::vector<DjkNode>> priority_queue{};

    double new_distance{ 0 };
    priority_queue.emplace(source, new_distance);
    while (!priority_queue.empty())  // The main loop
    {
      DjkNode u = priority_queue.top();
      Value u_value = u.value;
      if (u_value == to)
      {
        break;
      }
      priority_queue.pop();
      auto neighbors = get_neighbors(u_value);
      for (auto v : neighbors)  // Go through all v neighbors of u
      {
        new_distance = distances[u_value] + distance_function(u_value, v);
        if (distances.count(v) == 0 || new_distance < distances[v])
        {
          distances[v] = new_distance;
          previous[v] = u_value;
          priority_queue.emplace(v, new_distance);
        }
      }
    }
    // return dist, prev
    std::vector<Value> path;
    Value u{ to };
    if (previous.count(u) > 0)
    {
      while (u != source)
      {
        path.insert(path.begin(), u);
        u = previous[u];
      }
      path.insert(path.begin(), u);
    }
    return path;
  }
};
}  // namespace utilities
}  // namespace prx